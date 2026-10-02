package track

import "encoding/json"

// trackState is the serializable form of one live track.
type trackState struct {
	ID        int64     `json:"id"`
	Phase     phase     `json:"phase"`
	Hits      int       `json:"hits"`
	Misses    int       `json:"misses"`
	RangeWin  []float64 `json:"range_win"`
	VelWin    []float64 `json:"vel_win"`
	AmpWin    []float64 `json:"amp_win"`
	SNRWin    []float64 `json:"snr_win"`
	Head      int       `json:"head"`
	Filled    int       `json:"filled"`
	LastRange float64   `json:"last_range"`
	LastVel   float64   `json:"last_vel"`
	Amp       float64   `json:"amp"`
	SNR       float64   `json:"snr"`
}

type trackerState struct {
	Config Config       `json:"config"`
	NextID int64        `json:"next_id"`
	Tracks []trackState `json:"tracks"`
}

// MarshalState serializes the complete tracker state deterministically.
func (t *Tracker) MarshalState() ([]byte, error) {
	st := trackerState{Config: t.cfg, NextID: t.nextID}
	for _, tr := range t.live {
		st.Tracks = append(st.Tracks, trackState{
			ID:        tr.id,
			Phase:     tr.phase,
			Hits:      tr.hits,
			Misses:    tr.misses,
			RangeWin:  tr.rangeWin,
			VelWin:    tr.velWin,
			AmpWin:    tr.ampWin,
			SNRWin:    tr.snrWin,
			Head:      tr.head,
			Filled:    tr.filled,
			LastRange: tr.lastRange,
			LastVel:   tr.lastVel,
			Amp:       tr.amp,
			SNR:       tr.snr,
		})
	}
	return json.Marshal(&st)
}

// RestoreState rebuilds a tracker from bytes produced by MarshalState.
func RestoreState(data []byte) (*Tracker, error) {
	var st trackerState
	if err := json.Unmarshal(data, &st); err != nil {
		return nil, err
	}
	t := NewTracker(st.Config)
	t.nextID = st.NextID
	for _, s := range st.Tracks {
		t.live = append(t.live, &track{
			id:        s.ID,
			phase:     s.Phase,
			hits:      s.Hits,
			misses:    s.Misses,
			rangeWin:  s.RangeWin,
			velWin:    s.VelWin,
			ampWin:    s.AmpWin,
			snrWin:    s.SNRWin,
			head:      s.Head,
			filled:    s.Filled,
			lastRange: s.LastRange,
			lastVel:   s.LastVel,
			amp:       s.Amp,
			snr:       s.SNR,
		})
	}
	return t, nil
}
