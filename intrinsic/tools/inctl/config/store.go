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

// Package config manages inctl configuration shared across commands.
package config

import (
	"os"
	"path/filepath"

	"github.com/pkg/errors"
	"google.golang.org/protobuf/encoding/protojson"

	cpb "intrinsic/tools/inctl/config/config_go_proto"
)

const fileName = "config.json"

// Store manages the configuration file in the file system.
type Store struct {
	dir string
}

// Option configures a [Store].
type Option func(s *Store)

// WithDirectory sets the directory at which the config should be stored. The
// directory path must be absolute. If the directory does not exist yet the
// program must have permission to create it. If the directory already exists it
// must be writable.
func WithDirectory(dir string) Option {
	return func(s *Store) {
		s.dir = dir
	}
}

// NewStore creates a new [Store]. The store uses the system specific user home
// directory as its base directory. Returns an error if it cannot determine a
// user home. Defaults to {USER_CONFIG_DIR}/intrinsic/inctl.
func NewStore(opts ...Option) (*Store, error) {
	s := &Store{}
	for _, opt := range opts {
		opt(s)
	}
	if s.dir == "" {
		home, err := os.UserConfigDir()
		if err != nil {
			return nil, errors.Wrap(err, "could not get home directory")
		}
		s.dir = filepath.Join(home, "intrinsic/inctl")
	}
	return s, nil
}

// Path returns the fully qualified path to the configuration file. The
// configuration does not necessarily exist.
func (s *Store) Path() string {
	return filepath.Join(s.dir, fileName)
}

// Read returns the stored configuration. Returns an empty configuration and NO
// error if there is no configuration file.
func (s *Store) Read() (*cpb.Config, error) {
	b, err := os.ReadFile(s.Path())
	if err != nil {
		// Return an empty config if the config file does not exist. This is
		// explicitly not an error because not having a configuration is valid.
		if errors.Is(err, os.ErrNotExist) {
			return &cpb.Config{}, nil
		}
		return nil, errors.Wrap(err, "could not read config file")
	}
	cfg := &cpb.Config{}
	if err := protojson.Unmarshal(b, cfg); err != nil {
		return nil, errors.Wrap(err, "could not unmarshal config file")
	}
	return cfg, nil
}

// Write stores the given cfg in the file system.
func (s *Store) Write(cfg *cpb.Config) error {
	if err := os.MkdirAll(s.dir, 0o700); err != nil {
		return errors.Wrap(err, "could not create config directory")
	}
	b, err := protojson.Marshal(cfg)
	if err != nil {
		return errors.Wrap(err, "could not marshal config file")
	}
	if err := os.WriteFile(s.Path(), b, 0o644); err != nil {
		return errors.Wrap(err, "could not write config file")
	}
	return nil
}
