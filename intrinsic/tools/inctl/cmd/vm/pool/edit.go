// Copyright 2026 Intrinsic Innovation LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

package pool

import (
	"bytes"
	"context"
	"encoding/json"
	"errors"
	"fmt"
	"io"
	"os"
	"os/exec"
	"reflect"
	"strings"

	"intrinsic/tools/inctl/util/orgutil"
	"intrinsic/tools/inctl/util/promptutil"

	"cloud.google.com/go/storage"
	"github.com/spf13/cobra"
	"golang.org/x/term"
	"google.golang.org/api/iterator"
)

var (
	isTerminal = func(fd int) bool {
		return term.IsTerminal(fd)
	}
	newStorageClient = func(ctx context.Context) (*storage.Client, error) {
		return storage.NewClient(ctx)
	}
	runEditor = func(editor string, filePath string) error {
		editorCmd := exec.Command(editor, filePath)
		editorCmd.Stdin = os.Stdin
		editorCmd.Stdout = os.Stdout
		editorCmd.Stderr = os.Stderr
		return editorCmd.Run()
	}
	runDiff = func(originalPath string, modifiedPath string, w io.Writer) error {
		diffCmd := exec.Command("diff", "-u", originalPath, modifiedPath)
		diffCmd.Stdout = w
		diffCmd.Stderr = os.Stderr

		err := diffCmd.Run()
		if err != nil {
			var exitErr *exec.ExitError
			if errors.As(err, &exitErr) && exitErr.ExitCode() == 1 {
				return nil
			}
			return err
		}
		return nil
	}
)

var ErrEditAborted = errors.New("edit aborted by clearing editor content")

func NewEditCommand() *cobra.Command {
	var poolName string

	cmd := &cobra.Command{
		Use:   "edit",
		Short: "[Intrinsic Internal] Edit a VM pool configuration interactively.",
		Long:  "Resolves a VM pool configuration in GCS, opens it in $EDITOR, and uploads changes upon confirmation.",
		RunE: func(cmd *cobra.Command, args []string) error {
			if err := agents.Check(cmd); err != nil {
				return err
			}

			ctx := cmd.Context()

			if !isTerminal(int(os.Stdin.Fd())) {
				return fmt.Errorf("standard input is not a TTY terminal, interactive editing is not supported")
			}

			project := viperLocal.GetString(orgutil.KeyProject)
			if project == "" {
				return fmt.Errorf("project is required (use --project flag or set active context)")
			}

			gcsClient, err := newStorageClient(ctx)
			if err != nil {
				return fmt.Errorf("create GCS client: %w", err)
			}
			defer gcsClient.Close()

			bucketName := fmt.Sprintf("%s-vmpoolconf", project)
			finalGCSPath, err := resolvePoolConfigPath(ctx, gcsClient, bucketName, poolName)
			if err != nil {
				return err
			}

			data, err := readGCSObject(ctx, gcsClient, bucketName, finalGCSPath)
			if err != nil {
				return fmt.Errorf("download pool config: %w", err)
			}

			modifiedData, err := spawnInteractiveEditor(data)
			if err != nil {
				if errors.Is(err, ErrEditAborted) {
					fmt.Fprintln(cmd.OutOrStdout(), "Editor file empty. Discarding changes.")
					return nil
				}
				return err
			}

			equal, originalJSON, modifiedJSON, err := checkSemanticEquality(data, modifiedData)
			if err != nil {
				return err
			}
			if equal {
				fmt.Println("No changes detected.")
				return nil
			}

			if err := validateImmutableFields(data, modifiedData); err != nil {
				return err
			}

			if err := displayDiffPreview(originalJSON, modifiedJSON, cmd.OutOrStdout()); err != nil {
				return err
			}

			confirm, err := promptutil.PromptYesNo(cmd, "Do you want to upload these changes?", promptutil.DefaultNo, promptutil.ReemitPromptOnInvalidInput)
			if err != nil {
				return fmt.Errorf("prompt: %w", err)
			}

			if !confirm {
				fmt.Println("Discarding changes.")
				return nil
			}

			if err := uploadConfig(ctx, gcsClient, bucketName, finalGCSPath, modifiedData); err != nil {
				return err
			}

			fmt.Printf("Successfully uploaded changes to gs://%s/%s\n", bucketName, finalGCSPath)
			return nil
		},
	}

	cmd.Flags().StringVar(&poolName, "pool", "", poolFlagDesc)
	cmd.MarkFlagRequired("pool")
	return cmd
}

func resolveByBasename(ctx context.Context, gcsClient *storage.Client, bucketName, basename string) (string, error) {
	prefix := "live/pools/"
	it := gcsClient.Bucket(bucketName).Objects(ctx, &storage.Query{Prefix: prefix})

	var matches []string
	for {
		attrs, err := it.Next()
		if err == iterator.Done {
			break
		}
		if err != nil {
			return "", fmt.Errorf("list objects in GCS bucket %s: %w", bucketName, err)
		}

		if !strings.HasSuffix(attrs.Name, "/config.json") {
			continue
		}

		data, err := readGCSObject(ctx, gcsClient, bucketName, attrs.Name)
		if err != nil {
			continue
		}

		var config struct {
			Name   string            `json:"name"`
			Labels map[string]string `json:"labels"`
		}
		if err := json.Unmarshal(data, &config); err != nil {
			continue
		}

		if config.Labels["base_name"] == basename || config.Name == basename {
			matches = append(matches, attrs.Name)
		}
	}

	if len(matches) == 0 {
		return "", fmt.Errorf("pool %q not found (checked direct path and basenames)", basename)
	}

	if len(matches) > 1 {
		var fullnames []string
		for _, m := range matches {
			parts := strings.Split(m, "/")
			if len(parts) >= 3 {
				fullnames = append(fullnames, parts[2])
			} else {
				fullnames = append(fullnames, m)
			}
		}
		return "", fmt.Errorf("multiple pools found for basename %q: %v", basename, fullnames)
	}

	return matches[0], nil
}

func readGCSObject(ctx context.Context, gcsClient *storage.Client, bucket, object string) ([]byte, error) {
	rc, err := gcsClient.Bucket(bucket).Object(object).NewReader(ctx)
	if err != nil {
		return nil, err
	}
	defer rc.Close()
	return io.ReadAll(rc)
}

func resolvePoolConfigPath(ctx context.Context, gcsClient *storage.Client, bucketName, poolName string) (string, error) {
	resolvedPath := fmt.Sprintf("live/pools/%s/config.json", poolName)
	obj := gcsClient.Bucket(bucketName).Object(resolvedPath)
	_, err := obj.Attrs(ctx)

	if err == nil {
		return resolvedPath, nil
	} else if errors.Is(err, storage.ErrObjectNotExist) {
		return resolveByBasename(ctx, gcsClient, bucketName, poolName)
	}
	return "", fmt.Errorf("check GCS path gs://%s/%s: %w", bucketName, resolvedPath, err)
}

func spawnInteractiveEditor(originalData []byte) ([]byte, error) {
	tempFile, err := os.CreateTemp("", "vmpool-*.json")
	if err != nil {
		return nil, fmt.Errorf("create local temp file: %w", err)
	}
	tempFilePath := tempFile.Name()
	defer os.Remove(tempFilePath)

	if _, err := tempFile.Write(originalData); err != nil {
		tempFile.Close()
		return nil, fmt.Errorf("write to temp file: %w", err)
	}
	tempFile.Close()

	editor := os.Getenv("EDITOR")
	if editor == "" {
		if path, err := exec.LookPath("sensible-editor"); err == nil {
			editor = path
		} else if path, err := exec.LookPath("nano"); err == nil {
			editor = path
		} else if path, err := exec.LookPath("vi"); err == nil {
			editor = path
		} else {
			return nil, fmt.Errorf("no editor found ($EDITOR is not set, and sensible-editor/nano/vi are not in PATH)")
		}
	}

	if err := runEditor(editor, tempFilePath); err != nil {
		return nil, fmt.Errorf("editor: %w", err)
	}

	modifiedData, err := os.ReadFile(tempFilePath)
	if err != nil {
		return nil, fmt.Errorf("read modified temp file: %w", err)
	}

	if len(bytes.TrimSpace(modifiedData)) == 0 {
		return nil, ErrEditAborted
	}

	return modifiedData, nil
}

func validateImmutableFields(originalData, modifiedData []byte) error {
	var originalImmutable struct {
		Name     string `json:"name"`
		SpecHash string `json:"SpecHash"`
	}
	if err := json.Unmarshal(originalData, &originalImmutable); err != nil {
		return fmt.Errorf("parse original config for sanity checks: %w", err)
	}

	var modifiedImmutable struct {
		Name     string `json:"name"`
		SpecHash string `json:"SpecHash"`
	}
	if err := json.Unmarshal(modifiedData, &modifiedImmutable); err != nil {
		return fmt.Errorf("parse modified config for sanity checks: %w", err)
	}

	if originalImmutable.Name != modifiedImmutable.Name {
		return fmt.Errorf("field \"name\" is immutable (attempted to change %q to %q)", originalImmutable.Name, modifiedImmutable.Name)
	}
	if originalImmutable.SpecHash != modifiedImmutable.SpecHash {
		return fmt.Errorf("field \"SpecHash\" is immutable (attempted to change %q to %q)", originalImmutable.SpecHash, modifiedImmutable.SpecHash)
	}
	return nil
}

func displayDiffPreview(originalJSON, modifiedJSON interface{}, w io.Writer) error {
	oldPretty, _ := json.MarshalIndent(originalJSON, "", "  ")
	newPretty, _ := json.MarshalIndent(modifiedJSON, "", "  ")

	originalTempFile, err := os.CreateTemp("", "vmpool-original-*.json")
	if err != nil {
		return fmt.Errorf("create original temp file: %w", err)
	}
	originalTempFilePath := originalTempFile.Name()
	defer os.Remove(originalTempFilePath)

	if _, err := originalTempFile.Write(oldPretty); err != nil {
		originalTempFile.Close()
		return fmt.Errorf("write original temp file: %w", err)
	}
	originalTempFile.Close()

	modifiedTempFile, err := os.CreateTemp("", "vmpool-modified-*.json")
	if err != nil {
		return fmt.Errorf("create modified temp file: %w", err)
	}
	modifiedTempFilePath := modifiedTempFile.Name()
	defer os.Remove(modifiedTempFilePath)

	if _, err := modifiedTempFile.Write(newPretty); err != nil {
		modifiedTempFile.Close()
		return fmt.Errorf("write modified temp file: %w", err)
	}
	modifiedTempFile.Close()

	fmt.Fprintln(w, "--- Unified GCS Configuration Diff ---")
	if err := runDiff(originalTempFilePath, modifiedTempFilePath, w); err != nil {
		return fmt.Errorf("diff preview: %w", err)
	}
	fmt.Fprintln(w, "---------------------------------------")
	return nil
}

func uploadConfig(ctx context.Context, gcsClient *storage.Client, bucketName, gcsPath string, data []byte) error {
	wc := gcsClient.Bucket(bucketName).Object(gcsPath).NewWriter(ctx)
	wc.ContentType = "application/json"
	if _, err := wc.Write(data); err != nil {
		wc.Close()
		return fmt.Errorf("upload config to GCS: %w", err)
	}
	if err := wc.Close(); err != nil {
		return fmt.Errorf("finalize GCS upload: %w", err)
	}
	return nil
}

func checkSemanticEquality(originalData, modifiedData []byte) (bool, interface{}, interface{}, error) {
	var originalJSON interface{}
	if err := json.Unmarshal(originalData, &originalJSON); err != nil {
		return false, nil, nil, fmt.Errorf("unmarshal original config: %w", err)
	}

	var modifiedJSON interface{}
	if err := json.Unmarshal(modifiedData, &modifiedJSON); err != nil {
		return false, nil, nil, fmt.Errorf("unmarshal modified config: %w", err)
	}

	return reflect.DeepEqual(originalJSON, modifiedJSON), originalJSON, modifiedJSON, nil
}
