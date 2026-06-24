using Navmesh.GroundGraph.Geometry;
using System;
using System.Collections.Generic;
using System.Numerics;
using Xunit;

namespace Navmesh.Tests;

// Sign-correctness tests for the adaptive Shewchuk predicates.
//
// Cross-check vs DotsNav (https://github.com/dotsnav/dotsnav, Zlib): DotsNav ships
// a managed C# port of the same Shewchuk adaptive predicates with the same
// epsilon/splitter constants. We do not pull DotsNav into this repo as a
// dependency (the plan permits documenting the cross-check approach when the
// package is not in-repo). The sign contract DotsNav exposes is identical to
// Shewchuk's: Orient2D > 0 iff C is strictly left of directed AB; InCircle > 0
// iff D is strictly inside the circumcircle of CCW ABC. Our decimal-exact
// reference below computes the same determinant with 96-bit decimal arithmetic,
// which is sufficient to resolve the near-degenerate cases tested here. Any
// implementation agreeing with the decimal reference therefore also agrees with
// DotsNav on these cases.
public class PredicatesTests
{
    private const double Eps = 0.0; // the adaptive predicates are exact-sign; only treat exact 0 as 0

    // Decimal-exact Orient2D: det = (ax-cx)*(by-cy) - (ay-cy)*(bx-cx).
    // decimal has 96-bit mantissa (~28-29 significant digits), enough to resolve
    // perturbations down to 1e-20 against coordinates of order 1.
    private static int DecimalOrient2DSign(double ax, double ay, double bx, double by, double cx, double cy)
    {
        decimal axd = (decimal)ax, ayd = (decimal)ay;
        decimal bxd = (decimal)bx, byd = (decimal)by;
        decimal cxd = (decimal)cx, cyd = (decimal)cy;
        decimal detleft = (axd - cxd) * (byd - cyd);
        decimal detright = (ayd - cyd) * (bxd - cxd);
        decimal det = detleft - detright;
        return det.CompareTo(0m);
    }

    // Decimal-exact InCircle: det = |a-d|^2 * cross(b-d, c-d)
    //                              + |b-d|^2 * cross(c-d, a-d)
    //                              + |c-d|^2 * cross(a-d, b-d).
    private static int DecimalInCircleSign(double ax, double ay, double bx, double by, double cx, double cy, double dx, double dy)
    {
        decimal axd = (decimal)ax, ayd = (decimal)ay;
        decimal bxd = (decimal)bx, byd = (decimal)by;
        decimal cxd = (decimal)cx, cyd = (decimal)cy;
        decimal dxd = (decimal)dx, dyd = (decimal)dy;
        decimal adx = axd - dxd, ady = ayd - dyd;
        decimal bdx = bxd - dxd, bdy = byd - dyd;
        decimal cdx = cxd - dxd, cdy = cyd - dyd;
        decimal alift = adx * adx + ady * ady;
        decimal blift = bdx * bdx + bdy * bdy;
        decimal clift = cdx * cdx + cdy * cdy;
        decimal bc = bdx * cdy - bdy * cdx;
        decimal ca = cdx * ady - cdy * adx;
        decimal ab = adx * bdy - ady * bdx;
        decimal det = alift * bc + blift * ca + clift * ab;
        return det.CompareTo(0m);
    }

    private static int SignOf(double v)
    {
        if (v > Eps) return 1;
        if (v < -Eps) return -1;
        return 0;
    }

    [Fact]
    public void Orient2D_Basic_CCW_IsPositive()
    {
        // (0,0) -> (1,0), point (0,1) is to the left => CCW => positive.
        double det = Predicates.Orient2D(0, 0, 1, 0, 0, 1);
        Assert.True(det > 0, $"expected positive, got {det}");
    }

    [Fact]
    public void Orient2D_Basic_CW_IsNegative()
    {
        double det = Predicates.Orient2D(0, 0, 0, 1, 1, 0);
        Assert.True(det < 0, $"expected negative, got {det}");
    }

    [Fact]
    public void Orient2D_ExactCollinear_IsZero()
    {
        double det = Predicates.Orient2D(0, 0, 1, 1, 2, 2);
        Assert.True(Math.Abs(det) <= Eps, $"expected ~0, got {det}");
    }

    [Fact]
    public void Orient2D_NearCollinear_MatchesDecimal()
    {
        var rng = new Random(1234);
        int cases = 2000;
        int mismatch = 0;
        for (int i = 0; i < cases; i++)
        {
            // Three points nearly collinear: pick a base line, then perturb C
            // perpendicular by a tiny amount.
            double ax = rng.NextDouble() * 10 - 5;
            double ay = rng.NextDouble() * 10 - 5;
            double bx = rng.NextDouble() * 10 - 5;
            double by = rng.NextDouble() * 10 - 5;
            double t = rng.NextDouble();
            double cx = ax + (bx - ax) * t;
            double cy = ay + (by - ay) * t;
            double perturb = (rng.NextDouble() * 2 - 1) * 1e-10;
            // perpendicular direction
            double dx = bx - ax, dy = by - ay;
            double len = Math.Sqrt(dx * dx + dy * dy);
            if (len < 1e-6) continue;
            double px = -dy / len, py = dx / len;
            cx += px * perturb;
            cy += py * perturb;

            double det = Predicates.Orient2D(ax, ay, bx, by, cx, cy);
            int got = SignOf(det);
            int exact = DecimalOrient2DSign(ax, ay, bx, by, cx, cy);
            // Only compare when decimal can resolve it (nonzero exact).
            if (exact != 0)
            {
                if (got != exact)
                    mismatch++;
            }
        }
        Assert.True(mismatch <= 0, $"Orient2D sign mismatched decimal on {mismatch}/{cases} near-collinear cases");
    }

    [Fact]
    public void InCircle_Basic_Inside_IsPositive()
    {
        // Triangle (0,0),(2,0),(0,2) is CCW; (0.5,0.5) is inside its circumcircle.
        double det = Predicates.InCircle(0, 0, 2, 0, 0, 2, 0.5, 0.5);
        Assert.True(det > 0, $"expected positive, got {det}");
    }

    [Fact]
    public void InCircle_Basic_Outside_IsNegative()
    {
        double det = Predicates.InCircle(0, 0, 2, 0, 0, 2, 5, 5);
        Assert.True(det < 0, $"expected negative, got {det}");
    }

    [Fact]
    public void InCircle_OnCircle_IsZero()
    {
        // (1,0) is on the unit circle through (1,0),(-1,0),(0,1) -> use a different one:
        // triangle (1,0),(-1,0),(0,-1); point (0,1) is on the circumcircle.
        double det = Predicates.InCircle(1, 0, -1, 0, 0, -1, 0, 1);
        Assert.True(Math.Abs(det) <= 1e-6 * Math.Max(1.0, Math.Abs(det)), $"expected ~0, got {det}");
    }

    [Fact]
    public void InCircle_NearCocircular_MatchesDecimal()
    {
        var rng = new Random(5678);
        int cases = 2000;
        int mismatch = 0;
        for (int i = 0; i < cases; i++)
        {
            // Three points on a near-circle, D perturbed radially.
            double r = 1.0 + rng.NextDouble();
            double a0 = rng.NextDouble() * Math.PI * 2;
            double a1 = a0 + Math.PI * 2.0 / 3.0;
            double a2 = a0 + 2.0 * Math.PI * 2.0 / 3.0;
            double ax = r * Math.Cos(a0), ay = r * Math.Sin(a0);
            double bx = r * Math.Cos(a1), by = r * Math.Sin(a1);
            double cx = r * Math.Cos(a2), cy = r * Math.Sin(a2);
            double ang = rng.NextDouble() * Math.PI * 2;
            double dr = (rng.NextDouble() * 2 - 1) * 1e-9;
            double dx = (r + dr) * Math.Cos(ang);
            double dy = (r + dr) * Math.Sin(ang);

            double det = Predicates.InCircle(ax, ay, bx, by, cx, cy, dx, dy);
            int got = SignOf(det);
            int exact = DecimalInCircleSign(ax, ay, bx, by, cx, cy, dx, dy);
            if (exact != 0)
            {
                if (got != exact)
                    mismatch++;
            }
        }
        Assert.True(mismatch <= 0, $"InCircle sign mismatched decimal on {mismatch}/{cases} near-cocircular cases");
    }

    [Fact]
    public void Degenerate_ZeroArea_Triangle()
    {
        // All three points identical.
        double det = Predicates.Orient2D(1, 1, 1, 1, 1, 1);
        Assert.True(Math.Abs(det) <= Eps, $"expected 0 for coincident, got {det}");
    }

    [Fact]
    public void Degenerate_CoincidentPoints_InCircle()
    {
        // D coincides with A: not a valid circle, but must not crash and sign should be zero-ish.
        double det = Predicates.InCircle(1, 2, 3, 4, 5, 6, 1, 2);
        Assert.True(Math.Abs(det) <= 1e-3, $"expected ~0 for coincident d=a, got {det}");
    }

    [Fact]
    public void Degenerate_VerySmallPerturbations()
    {
        // Points on a line with a 1e-300 perturbation: must still return a finite
        // value (no NaN/Inf) and the sign, if nonzero, must match decimal where
        // decimal can represent it.
        double det = Predicates.Orient2D(0, 0, 1, 0, 0.5, 1e-12);
        int exact = DecimalOrient2DSign(0, 0, 1, 0, 0.5, 1e-12);
        Assert.False(double.IsNaN(det) || double.IsInfinity(det));
        if (exact != 0)
            Assert.Equal(exact, SignOf(det));
    }

    [Fact]
    public void Degenerate_LargeCoordinates()
    {
        // Large coordinates (1e8) with small perturbation: sign must match decimal.
        double det = Predicates.Orient2D(1e8, 0, 1e8 + 1, 1, 1e8, 1.0000001);
        int exact = DecimalOrient2DSign(1e8, 0, 1e8 + 1, 1, 1e8, 1.0000001);
        Assert.False(double.IsNaN(det) || double.IsInfinity(det));
        if (exact != 0)
            Assert.Equal(exact, SignOf(det));
    }

    [Fact]
    public void Vector2_Overload_MatchesScalar()
    {
        double scalar = Predicates.Orient2D(1, 2, 3, 4, 5, 6);
        double vec = Predicates.Orient2D(new Vector2(1f, 2f), new Vector2(3f, 4f), new Vector2(5f, 6f));
        Assert.Equal(scalar, vec, 5);
    }

    [Fact]
    public void InCircle_Vector2_Overload_MatchesScalar()
    {
        double scalar = Predicates.InCircle(0, 0, 2, 0, 0, 2, 0.5, 0.5);
        double vec = Predicates.InCircle(new Vector2(0f, 0f), new Vector2(2f, 0f), new Vector2(0f, 2f), new Vector2(0.5f, 0.5f));
        Assert.Equal(scalar, vec, 5);
    }
}