// Package store persists replay profiles to SQLite.
package store

import (
	"database/sql"
	"fmt"
	"time"

	_ "github.com/mattn/go-sqlite3"

	"github.com/xet-labs/roboarm-commander/internal/arm"
)

type Profile struct {
	ID        int64     `json:"id"`
	Name      string    `json:"name"`
	CreatedAt time.Time `json:"created_at"`
	Steps     []arm.Step `json:"steps"`
}

type Store struct {
	db *sql.DB
}

const schema = `
CREATE TABLE IF NOT EXISTS replay_profiles (
	id         INTEGER PRIMARY KEY AUTOINCREMENT,
	name       TEXT NOT NULL,
	created_at TEXT NOT NULL,
	steps_json TEXT NOT NULL
);
`

func Open(path string) (*Store, error) {
	db, err := sql.Open("sqlite3", path+"?_foreign_keys=on")
	if err != nil {
		return nil, fmt.Errorf("store: open: %w", err)
	}
	if _, err := db.Exec(schema); err != nil {
		return nil, fmt.Errorf("store: schema: %w", err)
	}
	return &Store{db: db}, nil
}

func (s *Store) Save(name string, steps []arm.Step) (int64, error) {
	stepsJSON, err := arm.MarshalSteps(steps)
	if err != nil {
		return 0, fmt.Errorf("store: marshal steps: %w", err)
	}
	res, err := s.db.Exec(
		`INSERT INTO replay_profiles (name, created_at, steps_json) VALUES (?, ?, ?)`,
		name, time.Now().UTC().Format(time.RFC3339), stepsJSON,
	)
	if err != nil {
		return 0, fmt.Errorf("store: insert: %w", err)
	}
	return res.LastInsertId()
}

func (s *Store) List() ([]Profile, error) {
	rows, err := s.db.Query(`SELECT id, name, created_at, steps_json FROM replay_profiles ORDER BY created_at DESC`)
	if err != nil {
		return nil, fmt.Errorf("store: list: %w", err)
	}
	defer rows.Close()

	var out []Profile
	for rows.Next() {
		var p Profile
		var createdAt, stepsJSON string
		if err := rows.Scan(&p.ID, &p.Name, &createdAt, &stepsJSON); err != nil {
			return nil, fmt.Errorf("store: scan: %w", err)
		}
		p.CreatedAt, _ = time.Parse(time.RFC3339, createdAt)
		p.Steps, err = arm.UnmarshalSteps(stepsJSON)
		if err != nil {
			return nil, fmt.Errorf("store: unmarshal steps for id=%d: %w", p.ID, err)
		}
		out = append(out, p)
	}
	return out, rows.Err()
}

func (s *Store) Get(id int64) (Profile, error) {
	var p Profile
	var createdAt, stepsJSON string
	err := s.db.QueryRow(
		`SELECT id, name, created_at, steps_json FROM replay_profiles WHERE id = ?`, id,
	).Scan(&p.ID, &p.Name, &createdAt, &stepsJSON)
	if err != nil {
		return Profile{}, fmt.Errorf("store: get id=%d: %w", id, err)
	}
	p.CreatedAt, _ = time.Parse(time.RFC3339, createdAt)
	p.Steps, err = arm.UnmarshalSteps(stepsJSON)
	if err != nil {
		return Profile{}, fmt.Errorf("store: unmarshal steps for id=%d: %w", id, err)
	}
	return p, nil
}

func (s *Store) Delete(id int64) error {
	_, err := s.db.Exec(`DELETE FROM replay_profiles WHERE id = ?`, id)
	return err
}

func (s *Store) Close() error {
	return s.db.Close()
}
