/*******************************************************************************
 * Copyright (c) 2026 Synecdoque
 *
 * The software is licensed under the MIT License. See the LICENSE file in this
 * repository for details.
 ***************************************************************************SDG*/

package main

import (
	"bytes"
	"fmt"
	"image"
	"image/color"
	"image/png"
	"math"

	"github.com/sdoque/mbaigo/forms"
)

// marginMetres is how much of the unseen around the seen ground the picture shows.
const marginMetres = 1.0

// mapPicture is the seen part of a map as a PNG, and where it lies. It carries its
// own geometry so that a picture and the numbers that place it can never come from
// different moments: the map grows, and its origin may move.
type mapPicture struct {
	PNG        []byte  `json:"png"`
	MinX       float64 `json:"minX"` // the map coordinate of the picture's left edge
	MaxY       float64 `json:"maxY"` // and of its top edge
	Resolution float64 `json:"resolution"`
	Width      int     `json:"width"`
	Height     int     `json:"height"`
	Frame      string  `json:"frame"`
	Generation int     `json:"generation"`
}

// render draws the map, row 0 of the picture being the top: the largest y.
func render(m *forms.MapA_v1a) (*mapPicture, error) {
	if err := m.Check(); err != nil {
		return nil, err
	}
	if m.Resolution <= 0 {
		return nil, fmt.Errorf("the map has no resolution")
	}
	x0, y0, x1, y1 := m.Width, m.Height, -1, -1
	for j := 0; j < m.Height; j++ {
		for i := 0; i < m.Width; i++ {
			if m.Cells[j*m.Width+i] != forms.MapUnknown {
				x0, x1 = min(x0, i), max(x1, i)
				y0, y1 = min(y0, j), max(y1, j)
			}
		}
	}
	if x1 < 0 {
		return nil, fmt.Errorf("nothing has been seen yet")
	}
	margin := int(math.Ceil(marginMetres / m.Resolution))
	x0, y0 = max(0, x0-margin), max(0, y0-margin)
	x1, y1 = min(m.Width-1, x1+margin), min(m.Height-1, y1+margin)

	const unseen, floor, wall = 0, 1, 2
	palette := color.Palette{color.Gray{Y: 200}, color.Gray{Y: 255}, color.Gray{Y: 20}}
	w, h := x1-x0+1, y1-y0+1
	img := image.NewPaletted(image.Rect(0, 0, w, h), palette)
	for j := y0; j <= y1; j++ {
		row := y1 - j
		for i := x0; i <= x1; i++ {
			var c uint8 = unseen
			switch v := m.Cells[j*m.Width+i]; {
			case v == forms.MapUnknown:
			case v >= 50:
				c = wall
			default:
				c = floor
			}
			img.SetColorIndex(i-x0, row, c)
		}
	}
	var buf bytes.Buffer
	if err := png.Encode(&buf, img); err != nil {
		return nil, err
	}
	return &mapPicture{
		PNG:        buf.Bytes(),
		MinX:       m.OriginX + float64(x0)*m.Resolution,
		MaxY:       m.OriginY + float64(y1+1)*m.Resolution,
		Resolution: m.Resolution,
		Width:      w,
		Height:     h,
		Frame:      m.Frame,
	}, nil
}
