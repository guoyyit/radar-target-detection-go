package radar

import "math"

// TrackerConfig controls cross-frame association.
type TrackerConfig struct {
	AssocRange    float64 // metres
	AssocVelocity float64 // metres per second
	ConfirmCount  int     // consecutive hits before output
	HoldFrames    int     // frames kept alive while unmatched
	WindowSize    int     // hit-streak / averaging window length
}

type trackState struct {
	id         int
	rangeHist  []float64
	velHist    []float64
	ampHist    []float64
	snrHist    []float64
	hitStreak  int
	missCount  int
	confirmed  bool
	firstFrame int64
	lastFrame  int64
}

// Tracker associates frame detections across frames and confirms/deletes tracks.
type Tracker struct {
	cfg    TrackerConfig
	tracks []*trackState
	nextID int
}

// NewTracker constructs an empty tracker.
func NewTracker(cfg TrackerConfig) *Tracker {
	if cfg.ConfirmCount <= 0 {
		cfg.ConfirmCount = 1
	}
	if cfg.HoldFrames < 0 {
		cfg.HoldFrames = 0
	}
	if cfg.WindowSize <= 0 {
		cfg.WindowSize = 1
	}
	return &Tracker{cfg: cfg, nextID: 1}
}

// Update feeds one frame of clustered detections and returns confirmed targets.
// Coasted (unmatched but within the hold period) tracks are emitted with their
// last estimated values.
func (t *Tracker) Update(frameID int64, dets []Detection) []Target {
	assignment := t.assign(dets)
	used := make([]bool, len(t.tracks))
	for detIdx, trkIdx := range assignment {
		trk := t.tracks[trkIdx]
		used[trkIdx] = true
		trk.missCount = 0
		trk.hitStreak++
		trk.lastFrame = frameID
		if trk.firstFrame == 0 {
			trk.firstFrame = frameID
		}
		t.pushHist(trk, dets[detIdx])
		if !trk.confirmed && trk.hitStreak >= t.cfg.ConfirmCount {
			trk.confirmed = true
		}
	}
	for i, trk := range t.tracks {
		if used[i] {
			continue
		}
		trk.missCount++
		trk.hitStreak = 0
	}

	alive := t.tracks[:0]
	var out []Target
	for _, trk := range t.tracks {
		if trk.missCount > t.cfg.HoldFrames {
			continue // delete
		}
		alive = append(alive, trk)
		if trk.confirmed {
			out = append(out, trk.toTarget(frameID))
		}
	}
	t.tracks = alive
	sortTargets(out)
	return out
}

func sortTargets(ts []Target) {
	for i := 1; i < len(ts); i++ {
		for j := i; j > 0 && ts[j].TrackID < ts[j-1].TrackID; j-- {
			ts[j-1], ts[j] = ts[j], ts[j-1]
		}
	}
}

// assign performs a deterministic greedy nearest-neighbour association ordered
// by ascending normalized distance.
func (t *Tracker) assign(dets []Detection) []int {
	assignment := make([]int, len(dets))
	for i := range assignment {
		assignment[i] = -1
	}
	if len(dets) == 0 || len(t.tracks) == 0 {
		for i := range dets {
			trk := &trackState{id: t.nextID}
			t.nextID++
			t.tracks = append(t.tracks, trk)
			assignment[i] = len(t.tracks) - 1
		}
		return assignment
	}
	type pair struct {
		d, tr int
		dist  float64
	}
	var pairs []pair
	for i, d := range dets {
		for j, tr := range t.tracks {
			dist := t.distance(d, tr)
			if dist <= 1 {
				pairs = append(pairs, pair{i, j, dist})
			}
		}
	}
	// Simple insertion sort keeps determinism without allocating large sorts.
	for i := 1; i < len(pairs); i++ {
		for j := i; j > 0 && pairs[j-1].dist > pairs[j].dist; j-- {
			pairs[j-1], pairs[j] = pairs[j], pairs[j-1]
		}
	}
	detTaken := make([]bool, len(dets))
	trkTaken := make([]bool, len(t.tracks))
	for _, p := range pairs {
		if detTaken[p.d] || trkTaken[p.tr] {
			continue
		}
		detTaken[p.d] = true
		trkTaken[p.tr] = true
		assignment[p.d] = p.tr
	}
	for i := range dets {
		if assignment[i] >= 0 {
			continue
		}
		trk := &trackState{id: t.nextID, hitStreak: 1}
		t.nextID++
		t.tracks = append(t.tracks, trk)
		assignment[i] = len(t.tracks) - 1
	}
	return assignment
}

func (t *Tracker) distance(d Detection, tr *trackState) float64 {
	dr := (d.Range - mean(tr.rangeHist)) / t.cfg.AssocRange
	dv := (d.Velocity - mean(tr.velHist)) / t.cfg.AssocVelocity
	return math.Hypot(dr, dv)
}

func (t *Tracker) pushHist(tr *trackState, d Detection) {
	w := t.cfg.WindowSize
	tr.rangeHist = pushBounded(tr.rangeHist, d.Range, w)
	tr.velHist = pushBounded(tr.velHist, d.Velocity, w)
	tr.ampHist = pushBounded(tr.ampHist, d.Amplitude, w)
	tr.snrHist = pushBounded(tr.snrHist, d.SNRDB, w)
}

func (tr *trackState) toTarget(frameID int64) Target {
	last := frameID
	if tr.lastFrame != 0 {
		last = tr.lastFrame
	}
	return Target{
		TrackID:    tr.id,
		Range:      mean(tr.rangeHist),
		Velocity:   mean(tr.velHist),
		Amplitude:  mean(tr.ampHist),
		SNRDB:      mean(tr.snrHist),
		FirstFrame: tr.firstFrame,
		LastFrame:  last,
	}
}

func pushBounded(hist []float64, v float64, cap int) []float64 {
	hist = append(hist, v)
	if len(hist) > cap {
		hist = hist[len(hist)-cap:]
	}
	return hist
}

func mean(xs []float64) float64 {
	if len(xs) == 0 {
		return 0
	}
	sum := 0.0
	for _, x := range xs {
		sum += x
	}
	return sum / float64(len(xs))
}
