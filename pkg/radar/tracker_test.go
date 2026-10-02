package radar

import "testing"

func det(r, v float64) Detection {
	return Detection{Range: r, Velocity: v, Amplitude: 10, SNRDB: 20}
}

func TestTrackerConfirmHoldDelete(t *testing.T) {
	tr := NewTracker(TrackerConfig{
		AssocRange: 1, AssocVelocity: 1,
		ConfirmCount: 3, HoldFrames: 2, WindowSize: 3,
	})
	if out := tr.Update(1, []Detection{det(10, 1)}); len(out) != 0 {
		t.Fatalf("unconfirmed target emitted: %+v", out)
	}
	if out := tr.Update(2, []Detection{det(10, 1)}); len(out) != 0 {
		t.Fatalf("target emitted after 2/3 hits")
	}
	out := tr.Update(3, []Detection{det(10, 1)})
	if len(out) != 1 || out[0].TrackID != 1 {
		t.Fatalf("target should be confirmed on 3rd hit: %+v", out)
	}
	// Miss one frame: held and re-emitted.
	out = tr.Update(4, nil)
	if len(out) != 1 {
		t.Fatalf("held target should be emitted: %+v", out)
	}
	// Miss again: still within hold (missCount 2 <= 2).
	out = tr.Update(5, nil)
	if len(out) != 1 {
		t.Fatalf("target should survive hold period: %+v", out)
	}
	// Third miss: deleted.
	if out := tr.Update(6, nil); len(out) != 0 {
		t.Fatalf("target should be deleted after hold: %+v", out)
	}
}

func TestTrackerAssociation(t *testing.T) {
	tr := NewTracker(TrackerConfig{
		AssocRange: 1.5, AssocVelocity: 1.0,
		ConfirmCount: 1, HoldFrames: 0, WindowSize: 3,
	})
	tr.Update(1, []Detection{det(10, 1), det(20, -1)})
	out := tr.Update(2, []Detection{det(10.3, 1.1), det(19.8, -0.9)})
	if len(out) != 2 {
		t.Fatalf("want 2 targets got %+v", out)
	}
	ids := map[int]int{}
	for _, tgt := range out {
		ids[tgt.TrackID]++
	}
	if ids[1] != 1 || ids[2] != 1 {
		t.Fatalf("track ids not preserved: %+v", out)
	}
}

func TestTrackerConsecutiveConfirmRequiresStreak(t *testing.T) {
	tr := NewTracker(TrackerConfig{
		AssocRange: 1, AssocVelocity: 1,
		ConfirmCount: 3, HoldFrames: 1, WindowSize: 3,
	})
	tr.Update(1, []Detection{det(10, 0)})
	tr.Update(2, []Detection{det(10, 0)})
	tr.Update(3, nil) // miss resets streak
	out := tr.Update(4, []Detection{det(10, 0)})
	if len(out) != 0 {
		t.Fatalf("streak should reset after miss: %+v", out)
	}
	out = tr.Update(5, []Detection{det(10, 0)})
	if len(out) != 0 {
		t.Fatalf("need 3 consecutive hits after reset")
	}
	out = tr.Update(6, []Detection{det(10, 0)})
	if len(out) != 1 {
		t.Fatalf("target should confirm on the 3rd consecutive hit: %+v", out)
	}
}
