package radar

import (
	"encoding/json"
	"errors"
	"io/fs"
	"os"
	"path/filepath"
)

// CheckpointStore persists processing progress atomically.
type CheckpointStore interface {
	Load() (Progress, error)
	Save(p Progress) error
}

// FileCheckpointStore persists progress as a small JSON file. Writes go to a
// temporary file in the same directory followed by an atomic rename, so a
// crash never leaves a half-written checkpoint.
type FileCheckpointStore struct {
	path string
}

// NewFileCheckpointStore opens/creates a checkpoint file store.
func NewFileCheckpointStore(path string) (*FileCheckpointStore, error) {
	if path == "" {
		return nil, errors.New("checkpoint path must not be empty")
	}
	if dir := filepath.Dir(path); dir != "." {
		if err := os.MkdirAll(dir, 0o755); err != nil {
			return nil, err
		}
	}
	return &FileCheckpointStore{path: path}, nil
}

func (s *FileCheckpointStore) Load() (Progress, error) {
	data, err := os.ReadFile(s.path)
	if err != nil {
		if errors.Is(err, fs.ErrNotExist) {
			return Progress{}, nil
		}
		return Progress{}, err
	}
	var p Progress
	if len(data) == 0 {
		return p, nil
	}
	if err := json.Unmarshal(data, &p); err != nil {
		return Progress{}, err
	}
	return p, nil
}

func (s *FileCheckpointStore) Save(p Progress) error {
	data, err := json.MarshalIndent(p, "", "  ")
	if err != nil {
		return err
	}
	dir := filepath.Dir(s.path)
	tmp, err := os.CreateTemp(dir, ".checkpoint-*.tmp")
	if err != nil {
		return err
	}
	tmpName := tmp.Name()
	defer os.Remove(tmpName)
	if _, err := tmp.Write(data); err != nil {
		tmp.Close()
		return err
	}
	if err := tmp.Sync(); err != nil {
		tmp.Close()
		return err
	}
	if err := tmp.Close(); err != nil {
		return err
	}
	return os.Rename(tmpName, s.path)
}

// MemoryCheckpointStore keeps progress in memory (useful for tests and for
// ephemeral runs that still need restart boundaries in-process).
type MemoryCheckpointStore struct {
	p       Progress
	written bool
}

func NewMemoryCheckpointStore() *MemoryCheckpointStore {
	return &MemoryCheckpointStore{}
}

func (s *MemoryCheckpointStore) Load() (Progress, error) {
	return s.p, nil
}

func (s *MemoryCheckpointStore) Save(p Progress) error {
	s.p = p
	s.written = true
	return nil
}
