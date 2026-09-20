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

// wrap_pbtxt takes in an input file and wraps it in an any proto in pbtxt
// format (assuming it was valid to begin with).  It doesn't try to parse the
// file since outputing text proto formats is deliberately non-deterministic in
// golang.
package main

import (
	"bufio"
	"flag"
	"fmt"
	"os"

	"intrinsic/production/intrinsic"

	log "github.com/golang/glog"
)

var (
	input  = flag.String("input", "", "Input file name")
	output = flag.String("output", "-", "Output file name")
)

func parseTextprotoFile() (string, []string, error) {
	inFile, err := os.Open(*input)
	if err != nil {
		return "", nil, fmt.Errorf("unable to open %q: %v", *input, err)
	}
	defer inFile.Close()

	scan := bufio.NewScanner(inFile)
	scan.Split(bufio.ScanLines)
	var lines []string
	for scan.Scan() {
		lines = append(lines, scan.Text())
	}

	if want, got := 3, len(lines); want > got {
		return "", nil, fmt.Errorf("expected at least %d lines from textproto header, got %d", want, got)
	}

	var protoFile string
	_, err = fmt.Sscanf(lines[0], "# proto-file: %s", &protoFile)
	if err != nil {
		return "", nil, fmt.Errorf("unable to read proto-file from textproto header: %v", err)
	}
	var protoMessage string
	_, err = fmt.Sscanf(lines[1], "# proto-message: %s", &protoMessage)
	if err != nil {
		return "", nil, fmt.Errorf("unable to read proto-file from textproto header: %v", err)
	}

	if lines[2] != "" {
		return "", nil, fmt.Errorf("expected empty line after header")
	}

	return protoMessage, lines[3:], nil
}

func wrapPBTXT() error {
	if *input == "" {
		return fmt.Errorf("input file not specified")
	}
	if *output == "" {
		return fmt.Errorf("output file not specified")
	}

	protoMessage, lines, err := parseTextprotoFile()
	if err != nil {
		return fmt.Errorf("unable to get lines in file: %v", err)
	}

	var outFile *os.File
	if *output == "-" {
		outFile = os.Stdout
	} else {
		outFile, err = os.Create(*output)
		if err != nil {
			return fmt.Errorf("unable to open %q: %v", *output, err)
		}
	}
	defer outFile.Close()

	header := `# proto-file: google/protobuf/any.proto
# proto-message: google.protobuf.Any

[type.googleapis.com/%s] {
`
	if _, err := outFile.WriteString(fmt.Sprintf(header, protoMessage)); err != nil {
		return fmt.Errorf("unable to write to output %q: %v", *output, err)
	}
	for _, line := range lines {
		if _, err := outFile.WriteString(fmt.Sprintf("  %s\n", line)); err != nil {
			return fmt.Errorf("unable to write to output %q: %v", *output, err)
		}
	}
	if _, err := outFile.WriteString("}\n"); err != nil {
		return fmt.Errorf("unable to write to output %q: %v", *output, err)
	}
	return nil
}

func main() {
	intrinsic.Init()
	if err := wrapPBTXT(); err != nil {
		log.Exitf("Unable to wrap file: %v", err)
	}
}
