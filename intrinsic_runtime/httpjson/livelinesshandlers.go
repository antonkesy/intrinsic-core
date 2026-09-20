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

// Package livelinesshandlers contains HTTP Handlers that work without requiring any other gRPC services.
package livelinesshandlers

import (
	"encoding/json"
	"log/slog"
	"math/rand"
	"net/http"

	"github.com/grpc-ecosystem/grpc-gateway/v2/runtime"
)

// JokeResponse defines the structure for the JSON response
type JokeResponse struct {
	Joke string `json:"joke"`
}

// RegisterLivelinessHandlers registers handlers that work without depending on any other gRPC service
func RegisterLivelinessHandlers(mux *runtime.ServeMux) {
	slog.Info("Registering liveliness handlers")
	mux.HandlePath("GET", "/jokes/v0/knockknock", knockKnockHandler)
}

func knockKnockHandler(w http.ResponseWriter, r *http.Request, _ map[string]string) {
	jokes := []string{
		"Knock knock. Who's there? E-stop. E-stop who? E-stop right there! You are violating the safety cage perimeter!",
		"Knock knock. Who's there? G-code. G-code who? Gee, code you please unjam the infeed?",
		"Knock knock. Who’s there? PLC. PLC who? PLC open the door, my end-effectors are full!",
		"Knock knock. Who’s there? Wire. Wire who? Wire you not automating this process yet?",
		"Knock knock. Who’s there? Sensor. Sensor who? Sensor you’re already here, can clear this fault?",
		"Knock knock. Who’s there? Servo. Servo who? Servo yourself some coffee while the robot does the work.",
		"Knock knock. Who’s there? Anne. Anne who? Anne-droid!",
		"Knock knock. Who’s there? Artie. Artie who? Artie-ficial Intelligence.",
		"Knock knock. Who’s there? Factory. Factory who? Factory reset. What were we doing?",
		"Knock knock. Who’s there? Justin. Justin who? Justin time delivery!",
		"Knock knock. Who’s there? ... ... Lag.",
	}

	randomIndex := rand.Intn(len(jokes))
	selectedJoke := jokes[randomIndex]

	resp := JokeResponse{
		Joke: selectedJoke,
	}

	w.Header().Set("Content-Type", "application/json")
	w.WriteHeader(http.StatusOK)

	if err := json.NewEncoder(w).Encode(resp); err != nil {
		http.Error(w, "Failed to encode joke", http.StatusInternalServerError)
	}
}
