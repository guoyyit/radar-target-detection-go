// Package frame defines the simulated baseband echo frame wire format and its
// strict header validation.
package frame

import (
	"encoding/binary"
	"errors"
	"io"
	"math"
)

// Magic prefixes every encoded frame.
const Magic = "RDRF"

// ProtocolVersion is the only accepted wire format version.
const ProtocolVersion = 1

const (
	// envelopeLen: magic(4) + frameLen(4) + ver(1) + flags(1) + headerLen(2).
	envelopeLen = 12
	// coreHeaderLen: seq(8) + 4 float64 params(32) + pulseCount(4) +
	// samplesPerPulse(4).
	coreHeaderLen = 48
	// wireHeaderLen is the full fixed header length.
	wireHeaderLen = envelopeLen + coreHeaderLen

	lePayloadFloats = 2 // I and Q, both float64
)

// Header carries the per-frame radar parameters.
type Header struct {
	SampleRate      float64 // Hz, fast-time sampling rate
	ChirpSlope      float64 // Hz/s, frequency slope of each linear chirp
	CarrierFreq     float64 // Hz, RF center frequency (wavelength for velocity)
	PulseInterval   float64 // s, pulse repetition interval (PRI)
	PulseCount      int     // chirps / slow-time pulses in this frame
	SamplesPerPulse int     // fast-time samples per pulse
}

// RawFrame is one decoded baseband echo frame. I and Q are indexed
// [pulse][sample] and share identical dimensions.
type RawFrame struct {
	Seq    int64
	Header Header
	I      [][]float64
	Q      [][]float64
}

// ParseConfig bounds accepted frames so malformed headers cannot trigger
// unbounded allocation. Zero limits are disabled except the default frame
// size guard.
type ParseConfig struct {
	MaxPulseCount      int
	MaxSamplesPerPulse int
	MaxFrameBytes      int
}

// DefaultMaxFrameBytes bounds a single encoded frame unless overridden.
const DefaultMaxFrameBytes = 256 << 20

// Distinguishable parse failure reasons.
var (
	ErrBadMagic           = errors.New("frame: bad magic prefix")
	ErrUnsupportedVersion = errors.New("frame: unsupported protocol version")
	ErrTruncatedHeader    = errors.New("frame: truncated header")
	ErrTruncatedPayload   = errors.New("frame: truncated payload")
	ErrInvalidSampleRate  = errors.New("frame: invalid sample rate")
	ErrInvalidChirpSlope  = errors.New("frame: invalid chirp slope")
	ErrInvalidCarrierFreq = errors.New("frame: invalid carrier frequency")
	ErrInvalidPRI         = errors.New("frame: invalid pulse interval")
	ErrInvalidPulseCount  = errors.New("frame: invalid pulse count")
	ErrInvalidSamples     = errors.New("frame: invalid samples per pulse")
	ErrPayloadMismatch    = errors.New("frame: payload size does not match header")
	ErrLimitExceeded      = errors.New("frame: parameter exceeds configured limit")
)

// ParseError wraps a sentinel reason with the offending field.
type ParseError struct {
	Reason error
	Field  string
}

func (e *ParseError) Error() string { return e.Reason.Error() + ": " + e.Field }
func (e *ParseError) Unwrap() error { return e.Reason }

// Parse decodes exactly one self-delimiting frame from r. It consumes exactly
// frameLen bytes, which makes back-to-back frames on one stream safe.
func Parse(r io.Reader, cfg ParseConfig) (*RawFrame, error) {
	var env [envelopeLen]byte
	if _, err := io.ReadFull(r, env[:]); err != nil {
		return nil, &ParseError{Reason: ErrTruncatedHeader, Field: "envelope"}
	}
	if string(env[0:4]) != Magic {
		return nil, &ParseError{Reason: ErrBadMagic, Field: "magic"}
	}
	frameLen := int(binary.LittleEndian.Uint32(env[4:8]))
	if env[8] != ProtocolVersion {
		return nil, &ParseError{Reason: ErrUnsupportedVersion, Field: "version"}
	}
	headerLen := int(binary.LittleEndian.Uint16(env[10:12]))
	if headerLen < coreHeaderLen || frameLen < envelopeLen+headerLen {
		return nil, &ParseError{Reason: ErrPayloadMismatch, Field: "frameLen"}
	}

	head := make([]byte, headerLen)
	if _, err := io.ReadFull(r, head); err != nil {
		return nil, &ParseError{Reason: ErrTruncatedHeader, Field: "header"}
	}

	f := &RawFrame{
		Seq: int64(binary.LittleEndian.Uint64(head[0:8])),
		Header: Header{
			SampleRate:      math.Float64frombits(binary.LittleEndian.Uint64(head[8:16])),
			ChirpSlope:      math.Float64frombits(binary.LittleEndian.Uint64(head[16:24])),
			CarrierFreq:     math.Float64frombits(binary.LittleEndian.Uint64(head[24:32])),
			PulseInterval:   math.Float64frombits(binary.LittleEndian.Uint64(head[32:40])),
			PulseCount:      int(binary.LittleEndian.Uint32(head[40:44])),
			SamplesPerPulse: int(binary.LittleEndian.Uint32(head[44:48])),
		},
	}
	if err := f.Header.validate(cfg); err != nil {
		return nil, err
	}

	sampleCount := f.Header.PulseCount * f.Header.SamplesPerPulse
	payloadLen := sampleCount * lePayloadFloats * 8
	if frameLen != wireHeaderLen+payloadLen {
		// Consume the declared body so r stays aligned with the next frame.
		bodyLen := frameLen - envelopeLen - headerLen
		if bodyLen > 0 {
			if _, err := io.CopyN(io.Discard, r, int64(bodyLen)); err != nil {
				return nil, &ParseError{Reason: ErrTruncatedPayload, Field: "discard"}
			}
		}
		return nil, &ParseError{Reason: ErrPayloadMismatch, Field: "payloadLen"}
	}

	payload := make([]byte, payloadLen)
	if _, err := io.ReadFull(r, payload); err != nil {
		return nil, &ParseError{Reason: ErrTruncatedPayload, Field: "iq"}
	}

	f.I = make([][]float64, f.Header.PulseCount)
	f.Q = make([][]float64, f.Header.SamplesPerPulse)
	off := 0
	for p := 0; p < f.Header.PulseCount; p++ {
		f.I[p] = make([]float64, f.Header.SamplesPerPulse)
		f.Q[p] = make([]float64, f.Header.SamplesPerPulse)
		for s := 0; s < f.Header.SamplesPerPulse; s++ {
			f.I[p][s] = math.Float64frombits(binary.LittleEndian.Uint64(payload[off : off+8]))
			off += 8
			f.Q[p][s] = math.Float64frombits(binary.LittleEndian.Uint64(payload[off : off+8]))
			off += 8
		}
	}
	return f, nil
}

// Encode serializes one self-delimiting frame.
func Encode(f *RawFrame) ([]byte, error) {
	if f == nil {
		return nil, &ParseError{Reason: ErrTruncatedPayload, Field: "nil"}
	}
	if err := f.Header.validate(ParseConfig{}); err != nil {
		return nil, err
	}
	if len(f.I) != f.Header.PulseCount || len(f.Q) != f.Header.PulseCount {
		return nil, &ParseError{Reason: ErrPayloadMismatch, Field: "pulseCount"}
	}
	for p := 0; p < f.Header.PulseCount; p++ {
		if len(f.I[p]) != f.Header.SamplesPerPulse || len(f.Q[p]) != f.Header.SamplesPerPulse {
			return nil, &ParseError{Reason: ErrPayloadMismatch, Field: "samplesPerPulse"}
		}
	}

	sampleCount := f.Header.PulseCount * f.Header.SamplesPerPulse
	payloadLen := sampleCount * lePayloadFloats * 8
	totalLen := wireHeaderLen + payloadLen
	if uint64(totalLen) > uint64(^uint32(0)) {
		return nil, &ParseError{Reason: ErrLimitExceeded, Field: "frameLen"}
	}

	buf := make([]byte, totalLen)
	copy(buf[0:4], Magic)
	binary.LittleEndian.PutUint32(buf[4:8], uint32(totalLen))
	buf[8] = ProtocolVersion
	buf[9] = 0
	binary.LittleEndian.PutUint16(buf[10:12], coreHeaderLen)
	binary.LittleEndian.PutUint64(buf[12:20], uint64(f.Seq))
	binary.LittleEndian.PutUint64(buf[20:28], math.Float64bits(f.Header.SampleRate))
	binary.LittleEndian.PutUint64(buf[28:36], math.Float64bits(f.Header.ChirpSlope))
	binary.LittleEndian.PutUint64(buf[36:44], math.Float64bits(f.Header.CarrierFreq))
	binary.LittleEndian.PutUint64(buf[44:52], math.Float64bits(f.Header.PulseInterval))
	binary.LittleEndian.PutUint32(buf[52:56], uint32(f.Header.PulseCount))
	binary.LittleEndian.PutUint32(buf[56:60], uint32(f.Header.SamplesPerPulse))

	off := wireHeaderLen
	for p := 0; p < f.Header.PulseCount; p++ {
		for s := 0; s < f.Header.SamplesPerPulse; s++ {
			binary.LittleEndian.PutUint64(buf[off:off+8], math.Float64bits(f.I[p][s]))
			off += 8
			binary.LittleEndian.PutUint64(buf[off:off+8], math.Float64bits(f.Q[p][s]))
			off += 8
		}
	}
	return buf, nil
}

func (h Header) validate(cfg ParseConfig) error {
	switch {
	case !finitePositive(h.SampleRate):
		return &ParseError{Reason: ErrInvalidSampleRate, Field: "sampleRate"}
	case !finitePositive(h.ChirpSlope):
		return &ParseError{Reason: ErrInvalidChirpSlope, Field: "chirpSlope"}
	case !finitePositive(h.CarrierFreq):
		return &ParseError{Reason: ErrInvalidCarrierFreq, Field: "carrierFreq"}
	case !finitePositive(h.PulseInterval):
		return &ParseError{Reason: ErrInvalidPRI, Field: "pulseInterval"}
	case h.PulseCount <= 0:
		return &ParseError{Reason: ErrInvalidPulseCount, Field: "pulseCount"}
	case h.SamplesPerPulse <= 0:
		return &ParseError{Reason: ErrInvalidSamples, Field: "samplesPerPulse"}
	}
	if cfg.MaxPulseCount > 0 && h.PulseCount > cfg.MaxPulseCount {
		return &ParseError{Reason: ErrLimitExceeded, Field: "pulseCount"}
	}
	if cfg.MaxSamplesPerPulse > 0 && h.SamplesPerPulse > cfg.MaxSamplesPerPulse {
		return &ParseError{Reason: ErrLimitExceeded, Field: "samplesPerPulse"}
	}
	maxBytes := cfg.MaxFrameBytes
	if maxBytes == 0 {
		maxBytes = DefaultMaxFrameBytes
	}
	if maxBytes > 0 {
		need := wireHeaderLen + h.PulseCount*h.SamplesPerPulse*lePayloadFloats*8
		if need > maxBytes {
			return &ParseError{Reason: ErrLimitExceeded, Field: "frameBytes"}
		}
	}
	return nil
}

func finitePositive(v float64) bool {
	return !math.IsNaN(v) && !math.IsInf(v, 0) && v > 0
}
