package main

import (
	"flag"
	"image"
	"image/color"
	"math"
	"math/rand"
	"time"

	"github.com/fogleman/gg"
)

// minimum size of a triangle edge before stopping recursion
const minSize float64 = 15

// minimum difference between a triangles colour and the source images colour before stopping recursion
const minDifference float64 = 40000

// Triangle containing 3 points
type Triangle struct {
	P1, P2, P3 Point
}

// area of triangle
func (t *Triangle) area() float64 {
	num1 := t.P1.X * (t.P2.Y - t.P3.Y)
	num2 := t.P2.X * (t.P3.Y - t.P1.Y)
	num3 := t.P3.X * (t.P1.Y - t.P2.Y)

	return math.Abs(0.5 * (num1 + num2 + num3))
}

// splits a triangle in 2
func (t *Triangle) split() []Triangle {

	distance1 := t.P1.distanceTo(t.P2)
	distance2 := t.P2.distanceTo(t.P3)
	distance3 := t.P1.distanceTo(t.P3)

	var rootPoint Point
	var newPoint Point
	var originalPoint1 Point
	var originalPoint2 Point

	switch {
	case distance1 > distance2 && distance1 > distance3:
		if distance1 < minSize {
			return make([]Triangle, 0)
		}
		rootPoint = t.P3
		newPoint = randomPointOnLine(t.P1, t.P2)
		originalPoint1 = t.P1
		originalPoint2 = t.P2
	case distance2 > distance1 && distance2 > distance3:
		if distance2 < minSize {
			return make([]Triangle, 0)
		}
		rootPoint = t.P1
		newPoint = randomPointOnLine(t.P2, t.P3)
		originalPoint1 = t.P2
		originalPoint2 = t.P3
	case distance3 > distance1 && distance3 > distance2:
		if distance3 < minSize {
			return make([]Triangle, 0)
		}
		rootPoint = t.P2
		newPoint = randomPointOnLine(t.P1, t.P3)
		originalPoint1 = t.P1
		originalPoint2 = t.P3
	}

	triangle1 := Triangle{rootPoint, newPoint, originalPoint1}
	triangle2 := Triangle{rootPoint, newPoint, originalPoint2}

	slice := make([]Triangle, 2)
	slice[0] = triangle1
	slice[1] = triangle2

	return slice
}

// Point containing X & Y
type Point struct {
	X, Y float64
}

// distance between points
func (p *Point) distanceTo(p2 Point) float64 {
	x1 := p.X
	x2 := p2.X
	y1 := p.Y
	y2 := p2.Y

	xSqrd := (x2 - x1) * (x2 - x1)
	ySqrd := (y2 - y1) * (y2 - y1)
	distance := math.Sqrt(xSqrd + ySqrd)

	return distance
}

// finds a random x, y value inside a triangle
func randomPixel(triangle Triangle) (float64, float64) {
	rand := rand.New(rand.NewSource(time.Now().UnixNano()))
	rx := rand.Float64()
	ry := rand.Float64()

	x := ((1 - math.Sqrt(rx)) * triangle.P1.X) + ((math.Sqrt(rx) * (1 - ry)) * triangle.P2.X) + ((math.Sqrt(rx) * ry) * triangle.P3.X)
	y := ((1 - math.Sqrt(rx)) * triangle.P1.Y) + ((math.Sqrt(rx) * (1 - ry)) * triangle.P2.Y) + ((math.Sqrt(rx) * ry) * triangle.P3.Y)

	return x, y
}

// Gets a random point on a line between two points, inside the middle third
func randomPointOnLine(point1, point2 Point) Point {
	r := rand.New(rand.NewSource(time.Now().UnixNano()))

	x1 := point1.X
	x2 := point2.X
	y1 := point1.Y
	y2 := point2.Y

	distance := point1.distanceTo(point2)
	randomDist := (r.Float64() * (distance / 3)) + (distance / 3)
	t := randomDist / distance

	x := ((1 - t) * x1) + (t * x2)
	y := ((1 - t) * y1) + (t * y2)

	return Point{x, y}
}

// fills a triangle based on the color found on a random pixel inside the triangle
func fill(triangle Triangle, dc *gg.Context, image image.Image) {
	x, y := randomPixel(triangle)

	pixel := image.At(int(x), int(y))

	dc.ClearPath()
	dc.NewSubPath()
	dc.SetFillRule(gg.FillRuleEvenOdd)
	dc.SetFillStyle(gg.NewSolidPattern(pixel))
	dc.MoveTo(triangle.P1.X, triangle.P1.Y)
	dc.LineTo(triangle.P2.X, triangle.P2.Y)
	dc.LineTo(triangle.P3.X, triangle.P3.Y)

	dc.ClosePath()
	dc.Fill()
}

// difference between 2 rgb colours
func colorDifference(color1, color2 color.Color) float64 {
	r1, g1, b1, _ := color1.RGBA()
	r2, g2, b2, _ := color2.RGBA()

	rVal := float64((r2 - r1) * (r2 - r2))
	gVal := float64((g2 - g1) * (g2 - g1))
	bVal := float64((b2 - b1) * (b2 - b1))

	return math.Sqrt(rVal + gVal + bVal)
}

// samples n amount of points inside triangle and compares to source image, returns the difference
func differenceBetween(triangle Triangle, image image.Image, dc *gg.Context) float64 {
	area := triangle.area()
	n := int(math.Min(20, math.Sqrt(area)))

	difference := float64(0)
	for i := 0; i < n; i++ {
		x, y := randomPixel(triangle)
		pixel := image.At(int(x), int(y))

		current := dc.Image()
		pixel2 := current.At(int(x), int(y))

		difference += colorDifference(pixel, pixel2)
	}

	return difference
}

// runs the algorithm
func runTriangulation(triangles []Triangle, dc *gg.Context, im image.Image, count int) int {

	newTriangles := make([]Triangle, 0)

	for _, v := range triangles {
		fill(v, dc, im)
		nextTriangles := v.split()
		for _, vTriangle := range nextTriangles {
			newTriangles = append(newTriangles, vTriangle)
		}
	}

	for _, v := range newTriangles {
		diff := differenceBetween(v, im, dc)
		if diff > minDifference {
			count++
			count = runTriangulation([]Triangle{v}, dc, im, count)
		}
	}
	return count
}

func main() {

	imagePtr := flag.String("image", "br.png", "image to run program against")
	outPtr := flag.String("out", "out.png", "output image name")
	flag.Parse()

	const W = 1200
	const H = 1200

	image, err := gg.LoadPNG(*imagePtr)
	if err != nil {
		panic(err)
	}

	r := rand.New(rand.NewSource(time.Now().UnixNano()))
	x := float64(r.Int31n(400) + 400)
	y := float64(r.Int31n(400) + 400)

	dc := gg.NewContext(W, H)
	dc.SetHexColor("#FFFFFF")
	dc.Clear()
	dc.SetRGBA(0, 0, 0, 1)
	dc.SetLineWidth(1)
	dc.MoveTo(0, 0)
	dc.LineTo(x, y)
	dc.LineTo(0, 1200)
	dc.MoveTo(1200, 1200)
	dc.LineTo(x, y)
	dc.LineTo(1200, 0)
	dc.Stroke()

	topTriangle := Triangle{Point{0, 0}, Point{1200, 0}, Point{x, y}}
	bottomTriangle := Triangle{Point{0, 1200}, Point{1200, 1200}, Point{x, y}}
	leftTriangle := Triangle{Point{0, 0}, Point{0, 1200}, Point{x, y}}
	rightTriangle := Triangle{Point{1200, 0}, Point{1200, 1200}, Point{x, y}}

	initialTriangles := []Triangle{topTriangle, bottomTriangle, leftTriangle, rightTriangle}
	runTriangulation(initialTriangles, dc, image, 0)

	dc.SavePNG(*outPtr)
}
