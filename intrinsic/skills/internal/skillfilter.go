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

// Package skillfilter allows working with filter strings (https://google.aip.dev/160) for skills.
package skillfilter

import (
	"errors"
	"fmt"
	"regexp"

	"google.golang.org/protobuf/proto"

	spb "intrinsic/skills/proto/skills_go_proto"
)

const (
	booleanTruePattern  = "(?:^%[1]s$)|(?:^%[1]s *= *true$)|(?:^%[1]s *!= *false$)"
	booleanFalsePattern = "(?:^(?:-|NOT )%[1]s$)|(?:^%[1]s *= *false$)|(?:^%[1]s *!= *true$)"
)

// ErrInvalidFilterString is returned by [Parse] if the filter string is invalid.
var ErrInvalidFilterString = errors.New("invalid filter string")

// Matcher matches skills against the filter string that was parsed to create it.
type Matcher struct {
	sideloaded *bool
}

// Match returns true if the skill matches the filter string.
func (m *Matcher) Match(skill *spb.Skill) bool {
	if m.sideloaded == nil {
		return true
	}
	// TODO: b/341003029 - We don't currently track the installation origin of pBTs. All pBTs are
	//   explicitly not matched if the sideloaded filter has a value.
	if skill.GetBehaviorTreeDescription() != nil {
		return false
	}
	if *m.sideloaded {
		return skill.GetSideloaded()
	}
	return !skill.GetSideloaded()
}

// Parse parses a filter string and returns a [Matcher] that can be used to match skills. The filter
// string must be a valid filter string as defined in https://google.aip.dev/160. Only a small
// subset of the specification is supported.
// * only the `sideloaded` field is evaluated
func Parse(filterString string) (*Matcher, error) {
	if filterString == "" {
		return &Matcher{}, nil
	}
	// We only support the single `sideloaded` field. This means we can use simple matching against
	// the entire filter string without needing to any splitting.
	trueMatch, err := regexp.MatchString(fmt.Sprintf(booleanTruePattern, "sideloaded"), filterString)
	if err != nil {
		return nil, fmt.Errorf("failed to match filter string %q: %w", filterString, err)
	} else if trueMatch {
		return &Matcher{
			sideloaded: proto.Bool(true),
		}, nil
	}
	falseMatch, err := regexp.MatchString(fmt.Sprintf(booleanFalsePattern, "sideloaded"), filterString)
	if err != nil {
		return nil, fmt.Errorf("failed to match filter string %q: %w", filterString, err)
	} else if falseMatch {
		return &Matcher{
			sideloaded: proto.Bool(false),
		}, nil
	}
	return nil, ErrInvalidFilterString
}
