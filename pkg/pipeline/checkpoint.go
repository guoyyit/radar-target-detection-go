package pipeline

import (
	"encoding/json"
	"errors"
	"os"
	"path/filepath"
)

// checkpointRecord is the durable state for one committed frame: the frame
// sequence number plus the tracker state reached after that frame.
type checkpointRecord struct {
	Seq     int64           `json:"seq"`
	Tracker json.RawMessage `json:"tracker"`
}

// CheckpointStore persists the state of the last frame whose result has
// already been emitted.
type CheckpointStore interface {
	Load() (*checkpointRecord, error)
	Save(rec checkpointRecord) error
	Close() error
}

// MemoryStore keeps progress in-process only.
type MemoryStore struct {
	rec *checkpointRecord
}

// NewMemoryStore returns an empty in-memory store.
func NewMemoryStore() *MemoryStore {
	return &MemoryStore{}
}

func (m *MemoryStore) Load() (*checkpointRecord, error) { return m.rec, nil }
func (m *MemoryStore) Save(rec checkpointRecord) error {
	cp := rec
	m.rec = &cp
	return nil
}
func (m *MemoryStore) Close() error { return nil }

// FileStore durably persists progress as a small JSON file written via
// temp-file-and-rename, so a crash can never leave a half-written checkpoint.
type FileStore struct {
	path string
	rec  *checkpointRecord
}

// NewFileStore loads any existing checkpoint at path; a missing file means a
// fresh start, while a present but unreadable/corrupt file is an error.
func NewFileStore(path string) (*FileStore, error) {
	f := &FileStore{path: path}
	if data, err := os.ReadFile(path); err == nil {
		var rec checkpointRecord
		if err := json.Unmarshal(data, &rec); err != nil {
			return nil, err
		}
		f.rec = &rec
	} else if !errors.Is(err, os.ErrNotExist) {
		return nil, err
	}
	return f, nil
}

func (f *FileStore) Load() (*checkpointRecord, error) { return f.rec, nil }

func (f *FileStore) Save(rec checkpointRecord) error {
	data, err := json.Marshal(&rec)
	if err != nil {
		return err
	}
	dir := filepath.Dir(f.path)
	tmp, err := os.CreateTemp(dir, ".checkpoint-*")
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
	if err := os.Rename(tmpName, f.path); err != nil {
		return err
	}
	cp := rec
	f.rec = &cp
	return nil
}

func (f *FileStore) Close() error { return nil }

// ErrFrameGap indicates an out-of-order or missing sequence during resume.
var ErrFrameGap = errors.New("pipeline: unexpected frame sequence gap")
