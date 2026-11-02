#!/usr/bin/env python3
"""
midi2buzzer.py — Convert a MIDI file into a C++ Note array for a passive buzzer.

Usage:
  python midi2buzzer.py song.mid                      # print array to console
  python midi2buzzer.py song.mid -o song.h            # save as header
  python midi2buzzer.py song.mid --wav preview.wav    # also render a buzzer-like preview
  python midi2buzzer.py song.mid --list-channels      # see which channels have notes
  python midi2buzzer.py song.mid --channel 0          # use only one channel (e.g. the melody)

Requires: pip install mido numpy
"""
import argparse
import wave
import mido
import numpy as np

NAMES = ["C", "Cs", "D", "Ds", "E", "F", "Fs", "G", "Gs", "A", "As", "B"]


def midi_to_hz(n):
    return 440.0 * 2 ** ((n - 69) / 12)


def note_name(n):
    return f"{NAMES[n % 12]}{n // 12 - 1}"


def list_channels(mid):
    counts = {}
    for msg in mid:
        if msg.type == "note_on" and msg.velocity > 0:
            counts[msg.channel] = counts.get(msg.channel, 0) + 1
    for ch, c in sorted(counts.items()):
        print(f"channel {ch}: {c} notes")


def extract(mid, channel=None, transpose=0):
    """Monophonic 'skyline': at any instant, play the highest active note."""
    active = set()
    segments = []          # (midi_note or None, seconds)
    t = 0.0
    seg_start = 0.0
    current = None

    def close(now):
        if now - seg_start > 1e-6:
            segments.append((current, now - seg_start))

    for msg in mid:                      # iterating a MidiFile gives time in seconds (tempo handled)
        t += msg.time
        if msg.type not in ("note_on", "note_off"):
            continue
        if channel is not None and msg.channel != channel:
            continue

        restrike = False
        if msg.type == "note_on" and msg.velocity > 0:
            if msg.note == current:
                restrike = True          # same note struck again -> keep as separate note
            active.add(msg.note)
        else:
            active.discard(msg.note)

        new = max(active) if active else None
        if new != current or restrike:
            close(t)
            current, seg_start = new, t

    close(t)
    # drop leading silence
    while segments and segments[0][0] is None:
        segments.pop(0)
    return [((n + transpose) if n is not None else None, d) for n, d in segments]


def to_cpp(segments, name):
    lines = [f"const Note {name}[] = {{"]
    for n, d in segments:
        ms = max(1, round(d * 1000))
        if n is None:
            lines.append(f"  {{0, {ms}}},        // rest")
        else:
            lines.append(f"  {{{round(midi_to_hz(n))}, {ms}}},  // {note_name(n)}")
    lines.append("};")
    lines.append(f"const size_t {name}_len = sizeof({name}) / sizeof({name}[0]);")
    return "\n".join(lines)


def render_wav(segments, path, gap_ms=20, sr=44100):
    """Square-wave preview, same gap between notes as the buzzer library."""
    out = []
    for n, d in segments:
        total = int(d * sr)
        gap = min(int(gap_ms / 1000 * sr), total)
        on = total - gap
        if n is None:
            out.append(np.zeros(total))
        else:
            t = np.arange(on) / sr
            sq = 0.3 * np.sign(np.sin(2 * np.pi * midi_to_hz(n) * t))
            out.append(np.concatenate([sq, np.zeros(gap)]))
    audio = (np.concatenate(out) * 32767).astype(np.int16)
    with wave.open(path, "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(sr)
        w.writeframes(audio.tobytes())


def main():
    p = argparse.ArgumentParser()
    p.add_argument("midi")
    p.add_argument("-o", "--out", help="output .h file")
    p.add_argument("--name", default="song", help="C++ array name")
    p.add_argument("--channel", type=int, help="only use this MIDI channel")
    p.add_argument("--transpose", type=int, default=0, help="semitones (+12 = one octave up)")
    p.add_argument("--wav", help="render a square-wave preview")
    p.add_argument("--list-channels", action="store_true")
    a = p.parse_args()

    mid = mido.MidiFile(a.midi)
    if a.list_channels:
        list_channels(mid)
        return

    segs = extract(mid, a.channel, a.transpose)
    code = to_cpp(segs, a.name)

    if a.out:
        with open(a.out, "w") as f:
            f.write("#pragma once\n#include \"Buzzer.h\"\n\n" + code + "\n")
        print(f"{len(segs)} notes -> {a.out}")
    else:
        print(code)

    if a.wav:
        render_wav(segs, a.wav)
        print(f"preview -> {a.wav}")


if __name__ == "__main__":
    main()
