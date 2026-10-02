package radar

import (
	"path/filepath"
	"testing"
)

func TestFileCheckpointRoundTrip(t *testing.T) {
	path := filepath.Join(t.TempDir(), "ckpt.json")
	store, err := NewFileCheckpointStore(path)
	if err != nil {
		t.Fatal(err)
	}
	p, err := store.Load()
	if err != nil {
		t.Fatal(err)
	}
	if p.LastCompletedFrame != 0 {
		t.Fatalf("fresh checkpoint = %d, want 0", p.LastCompletedFrame)
	}
	if err := store.Save(Progress{LastCompletedFrame: 17}); err != nil {
		t.Fatal(err)
	}
	store2, err := NewFileCheckpointStore(path)
	if err != nil {
		t.Fatal(err)
	}
	p, err = store2.Load()
	if err != nil {
		t.Fatal(err)
	}
	if p.LastCompletedFrame != 17 {
		t.Fatalf("loaded checkpoint = %d, want 17", p.LastCompletedFrame)
	}
}

func TestFileCheckpointAtomicNoTempLeft(t *testing.T) {
	dir := t.TempDir()
	path := filepath.Join(dir, "ckpt.json")
	store, err := NewFileCheckpointStore(path)
	if err != nil {
		t.Fatal(err)
	}
	for i := int64(1); i <= 5; i++ {
		if err := store.Save(Progress{LastCompletedFrame: i}); err != nil {
			t.Fatal(err)
		}
	}
	matches, err := filepath.Glob(filepath.Join(dir, "*.tmp"))
	if err != nil {
		t.Fatal(err)
	}
	if len(matches) != 0 {
		t.Fatalf("temporary checkpoint files left behind: %v", matches)
	}
}
