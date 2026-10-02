package radar

// FrameHeader carries the per-frame signal parameters.
type FrameHeader struct {
	SampleRateHz    float64
	ChirpSlope      float64 // Hz per second
	PulseCount      int
	SamplesPerPulse int
	CarrierFreqHz   float64
	PRISec          float64 // pulse repetition interval in seconds
}

// Pulse holds the baseband I/Q samples of one chirp.
type Pulse struct {
	I []float64
	Q []float64
}

// RawFrame is the unvalidated frame as received from the pipe.
type RawFrame struct {
	ID     int64
	Header FrameHeader
	Pulses []Pulse
}

// Frame is a validated echo frame.
type Frame struct {
	ID     int64
	Header FrameHeader
	Pulses []Pulse
}

// Detection is a single (possibly unclustered) target measurement.
type Detection struct {
	RangeBin   int
	DopplerBin int
	Range      float64
	Velocity   float64
	Amplitude  float64
	Noise      float64
	SNRDB      float64
}

// Target is a confirmed, tracked target.
type Target struct {
	TrackID    int
	Range      float64
	Velocity   float64
	Amplitude  float64
	SNRDB      float64
	FirstFrame int64
	LastFrame  int64
}

// RangeDoppler is the 2-D magnitude spectrum with physical axes.
type RangeDoppler struct {
	Magnitude   [][]float64 // [dopplerBin][rangeBin]
	RangeAxis   []float64   // metres per range bin
	DopplerAxis []float64   // m/s per doppler bin, FFT order
}

// Progress is the persisted processing checkpoint.
type Progress struct {
	LastCompletedFrame int64 `json:"last_completed_frame"`
}
