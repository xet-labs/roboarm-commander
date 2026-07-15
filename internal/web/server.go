package web

import (
	"embed"
	"encoding/json"
	"fmt"
	"io/fs"
	"log"
	"net/http"
	"strconv"

	"github.com/xet-labs/roboarm-commander/internal/arm"
	"github.com/xet-labs/roboarm-commander/internal/store"
)

//go:embed static/*
var staticFiles embed.FS

type Server struct {
	arm   *arm.Arm
	store *store.Store
	mux   *http.ServeMux
}

func New(a *arm.Arm, s *store.Store) *Server {
	srv := &Server{arm: a, store: s, mux: http.NewServeMux()}
	srv.routes()
	return srv
}

func (s *Server) ListenAndServe(addr string) error {
	log.Printf("[web] listening on %s", addr)
	return http.ListenAndServe(addr, s.mux)
}

func (s *Server) routes() {
	sub, err := fs.Sub(staticFiles, "static")
	if err != nil {
		log.Fatalf("[web] static assets: %v", err)
	}
	s.mux.Handle("/", http.FileServer(http.FS(sub)))

	s.mux.HandleFunc("/api/stats", s.handleStats)
	s.mux.HandleFunc("/api/mode", s.handleSetMode)
	s.mux.HandleFunc("/api/jog", s.handleJog)
	s.mux.HandleFunc("/api/claw", s.handleClaw)
	s.mux.HandleFunc("/api/home", s.handleHome)
	s.mux.HandleFunc("/api/stop", s.handleStop)

	s.mux.HandleFunc("/api/record/start", s.handleRecordStart)
	s.mux.HandleFunc("/api/record/stop", s.handleRecordStop)

	s.mux.HandleFunc("/api/profiles", s.handleProfiles)
	s.mux.HandleFunc("/api/profiles/export", s.handleExport)
	s.mux.HandleFunc("/api/profiles/import", s.handleImport)
	s.mux.HandleFunc("/api/replay/start", s.handleReplayStart)
	s.mux.HandleFunc("/api/replay/stop", s.handleReplayStop)
}

func writeJSON(w http.ResponseWriter, v interface{}) {
	w.Header().Set("Content-Type", "application/json")
	if err := json.NewEncoder(w).Encode(v); err != nil {
		log.Printf("[web] encode error: %v", err)
	}
}

func writeErr(w http.ResponseWriter, code int, err error) {
	w.WriteHeader(code)
	writeJSON(w, map[string]string{"error": err.Error()})
}

// GET /api/stats
func (s *Server) handleStats(w http.ResponseWriter, r *http.Request) {
	writeJSON(w, s.arm.Stats())
}

// POST /api/mode {"mode":"idle"|"live"|"replay"}
func (s *Server) handleSetMode(w http.ResponseWriter, r *http.Request) {
	if r.Method != http.MethodPost {
		writeErr(w, http.StatusMethodNotAllowed, fmt.Errorf("POST only"))
		return
	}
	var body struct {
		Mode arm.Mode `json:"mode"`
	}
	if err := json.NewDecoder(r.Body).Decode(&body); err != nil {
		writeErr(w, http.StatusBadRequest, err)
		return
	}
	if err := s.arm.SetMode(body.Mode); err != nil {
		writeErr(w, http.StatusBadRequest, err)
		return
	}
	writeJSON(w, s.arm.Stats())
}

// POST /api/jog {"base":0,"shoulder":0,"elbow":0,"wrist":0} — manual
// on-screen jog buttons; the Xbox bridge calls arm.Jog directly via the
// xbox package, not through this endpoint.
func (s *Server) handleJog(w http.ResponseWriter, r *http.Request) {
	var body struct{ Base, Shoulder, Elbow, Wrist int }
	if err := json.NewDecoder(r.Body).Decode(&body); err != nil {
		writeErr(w, http.StatusBadRequest, err)
		return
	}
	s.arm.Jog(body.Base, body.Shoulder, body.Elbow, body.Wrist)
	writeJSON(w, s.arm.Stats())
}

// POST /api/claw {"direction":-1|0|1,"duty":180}
func (s *Server) handleClaw(w http.ResponseWriter, r *http.Request) {
	var body struct {
		Direction int8
		Duty      byte
	}
	if err := json.NewDecoder(r.Body).Decode(&body); err != nil {
		writeErr(w, http.StatusBadRequest, err)
		return
	}
	s.arm.ClawSet(body.Direction, body.Duty)
	w.WriteHeader(http.StatusNoContent)
}

func (s *Server) handleHome(w http.ResponseWriter, r *http.Request) {
	s.arm.Home()
	writeJSON(w, s.arm.Stats())
}

func (s *Server) handleStop(w http.ResponseWriter, r *http.Request) {
	s.arm.EmergencyStop()
	writeJSON(w, s.arm.Stats())
}

func (s *Server) handleRecordStart(w http.ResponseWriter, r *http.Request) {
	if err := s.arm.StartRecording(); err != nil {
		writeErr(w, http.StatusBadRequest, err)
		return
	}
	writeJSON(w, s.arm.Stats())
}

// POST /api/record/stop {"name":"pick-and-place-1"}
func (s *Server) handleRecordStop(w http.ResponseWriter, r *http.Request) {
	var body struct{ Name string }
	_ = json.NewDecoder(r.Body).Decode(&body) // name optional, default below
	if body.Name == "" {
		body.Name = "untitled"
	}

	steps := s.arm.StopRecording()
	if len(steps) == 0 {
		writeErr(w, http.StatusBadRequest, fmt.Errorf("no steps recorded"))
		return
	}

	id, err := s.store.Save(body.Name, steps)
	if err != nil {
		writeErr(w, http.StatusInternalServerError, err)
		return
	}
	writeJSON(w, map[string]interface{}{"id": id, "steps": len(steps)})
}

func (s *Server) handleProfiles(w http.ResponseWriter, r *http.Request) {
	if r.Method == http.MethodDelete {
		id, err := strconv.ParseInt(r.URL.Query().Get("id"), 10, 64)
		if err != nil {
			writeErr(w, http.StatusBadRequest, err)
			return
		}
		if err := s.store.Delete(id); err != nil {
			writeErr(w, http.StatusInternalServerError, err)
			return
		}
		w.WriteHeader(http.StatusNoContent)
		return
	}

	profiles, err := s.store.List()
	if err != nil {
		writeErr(w, http.StatusInternalServerError, err)
		return
	}
	writeJSON(w, profiles)
}

// GET /api/profiles/export?id=3
func (s *Server) handleExport(w http.ResponseWriter, r *http.Request) {
	id, err := strconv.ParseInt(r.URL.Query().Get("id"), 10, 64)
	if err != nil {
		writeErr(w, http.StatusBadRequest, err)
		return
	}
	p, err := s.store.Get(id)
	if err != nil {
		writeErr(w, http.StatusNotFound, err)
		return
	}
	w.Header().Set("Content-Type", "text/csv")
	w.Header().Set("Content-Disposition", fmt.Sprintf(`attachment; filename="%s.csv"`, p.Name))
	if err := exportCSV(w, p.Steps); err != nil {
		log.Printf("[web] export error: %v", err)
	}
}

// POST /api/profiles/import (multipart form: file=<csv>, name=<optional>)
func (s *Server) handleImport(w http.ResponseWriter, r *http.Request) {
	if err := r.ParseMultipartForm(4 << 20); err != nil {
		writeErr(w, http.StatusBadRequest, err)
		return
	}
	file, header, err := r.FormFile("file")
	if err != nil {
		writeErr(w, http.StatusBadRequest, err)
		return
	}
	defer file.Close()

	steps, err := importCSV(file)
	if err != nil {
		writeErr(w, http.StatusBadRequest, err)
		return
	}

	name := r.FormValue("name")
	if name == "" {
		name = header.Filename
	}

	id, err := s.store.Save(name, steps)
	if err != nil {
		writeErr(w, http.StatusInternalServerError, err)
		return
	}
	writeJSON(w, map[string]interface{}{"id": id, "steps": len(steps)})
}

// POST /api/replay/start {"id":3}
func (s *Server) handleReplayStart(w http.ResponseWriter, r *http.Request) {
	var body struct{ ID int64 }
	if err := json.NewDecoder(r.Body).Decode(&body); err != nil {
		writeErr(w, http.StatusBadRequest, err)
		return
	}
	p, err := s.store.Get(body.ID)
	if err != nil {
		writeErr(w, http.StatusNotFound, err)
		return
	}
	if err := s.arm.SetMode(arm.ModeReplay); err != nil {
		writeErr(w, http.StatusBadRequest, err)
		return
	}
	if err := s.arm.Replay(p.Steps); err != nil {
		writeErr(w, http.StatusBadRequest, err)
		return
	}
	writeJSON(w, s.arm.Stats())
}

func (s *Server) handleReplayStop(w http.ResponseWriter, r *http.Request) {
	s.arm.StopReplay()
	writeJSON(w, s.arm.Stats())
}
