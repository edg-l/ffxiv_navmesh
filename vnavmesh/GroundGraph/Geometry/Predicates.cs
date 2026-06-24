// Adapted from Jonathan Richard Shewchuk, "Adaptive Precision Floating-Point
// Arithmetic and Fast Robust Geometric Predicates", Computational Geometry:
// Theory and Applications, and the public-domain reference implementation at
// https://www.cs.cmu.edu/~quake/robust.html (predicates.c by Shewchuk).
//
// Original code is public domain; this managed C# port retains that license.
// Implements the adaptive Orient2D and InCircle predicates using double-based
// expansion arithmetic, faithfully translated from predicates.c. The INEXACT
// qualifier in the C original marks values that may carry rounding error; in
// managed C# double arithmetic every floating-point operation may round, so
// the distinction is moot and we omit it.

using System;
using System.Runtime.CompilerServices;

namespace Navmesh.GroundGraph.Geometry;

public static class Predicates
{
    // IEEE 754 double: 53-bit significand. Shewchuk constants from predicates.c.
    private const double Epsilon = 1.1102230246251565e-16;        // 2^-53 (one ulp)
    private const double Splitter = 134217729.0;                  // 2^27 + 1

    // Error bounds from predicates.c. Orient2D uses ccwerrboundA (first, fast),
    // ccwerrboundB (adaptive), ccwerrboundC (tail refinement). InCircle uses
    // iccerrboundA, iccerrboundB, iccerrboundC analogously. resulterrbound is
    // the generic estimate-roundoff bound.
    private const double CcwErrBoundA = (3.0 + 16.0 * Epsilon) * Epsilon;
    private const double CcwErrBoundB = (2.0 + 12.0 * Epsilon) * Epsilon;
    private const double CcwErrBoundC = (9.0 + 64.0 * Epsilon) * Epsilon * Epsilon;
    private const double IccErrBoundA = (10.0 + 96.0 * Epsilon) * Epsilon;
    private const double IccErrBoundB = (4.0 + 48.0 * Epsilon) * Epsilon;
    private const double IccErrBoundC = (44.0 + 576.0 * Epsilon) * Epsilon * Epsilon;
    private const double ResultErrBound = (3.0 + 8.0 * Epsilon) * Epsilon;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static double Absolute(double a) => a >= 0.0 ? a : -a;

    // Fast_Two_Sum: assumes |a| >= |b|. Returns (x=a+b, y=roundoff).
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static (double x, double y) FastTwoSum(double a, double b)
    {
        double x = a + b;
        double bvirt = x - a;
        double y = b - bvirt;
        return (x, y);
    }

    // Two_Sum: no ordering assumption. Returns (x=a+b, y=roundoff).
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static (double x, double y) TwoSum(double a, double b)
    {
        double x = a + b;
        double bvirt = x - a;
        double avirt = x - bvirt;
        double bround = b - bvirt;
        double around = a - avirt;
        double y = around + bround;
        return (x, y);
    }

    // Two_Diff: returns (x=a-b, y=roundoff).
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static (double x, double y) TwoDiff(double a, double b)
    {
        double x = a - b;
        double bvirt = a - x;
        double avirt = x + bvirt;
        double bround = bvirt - b;
        double around = a - avirt;
        double y = around + bround;
        return (x, y);
    }

    // Two_Diff_Tail: given a, b, and x = a - b (already computed), return the
    // roundoff. Used by orient2dadapt/incircleadapt to recover the tail of a
    // subtraction that was already performed in rounded arithmetic.
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static double TwoDiffTail(double a, double b, double x)
    {
        double bvirt = a - x;
        double avirt = x + bvirt;
        double bround = bvirt - b;
        double around = a - avirt;
        return around + bround;
    }

    // Split: split a into (ahi, alo) with |alo| <= ahi * 2^-27.
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void Split(double a, out double ahi, out double alo)
    {
        double c = Splitter * a;
        double abig = c - a;
        ahi = c - abig;
        alo = a - ahi;
    }

    // Two_Product: returns (x=a*b, y=roundoff).
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static (double x, double y) TwoProduct(double a, double b)
    {
        double x = a * b;
        Split(a, out var ahi, out var alo);
        Split(b, out var bhi, out var blo);
        double err1 = x - (ahi * bhi);
        double err2 = err1 - (alo * bhi);
        double err3 = err2 - (ahi * blo);
        double y = (alo * blo) - err3;
        return (x, y);
    }

    // Square: returns (x=a*a, y=roundoff). Faster than Two_Product for a==b.
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static (double x, double y) Square(double a)
    {
        double x = a * a;
        Split(a, out var ahi, out var alo);
        double err1 = x - (ahi * ahi);
        double err3 = err1 - ((ahi + ahi) * alo);
        double y = (alo * alo) - err3;
        return (x, y);
    }

    // Two_Two_Diff: subtracts the 2-term expansion (b1,b0) from (a1,a0),
    // producing a 4-term expansion (x3,x2,x1,x0) in nondecreasing magnitude.
    // Unrolled: Two_One_Diff(a1,a0,b0,_j,_0,x0); Two_One_Diff(_j,_0,b1,x3,x2,x1).
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static (double x3, double x2, double x1, double x0) TwoTwoDiff(double a1, double a0, double b1, double b0)
    {
        var (_i, x0) = TwoDiff(a0, b0);
        var (_j, _0) = TwoSum(a1, _i);
        var (_i2, x1) = TwoDiff(_0, b1);
        var (x3, x2) = TwoSum(_j, _i2);
        return (x3, x2, x1, x0);
    }

    // Two_Two_Sum: sums the two 2-term expansions, 4-term result (x3,x2,x1,x0).
    // Unrolled: Two_One_Sum(a1,a0,b0,_j,_0,x0); Two_One_Sum(_j,_0,b1,x3,x2,x1).
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static (double x3, double x2, double x1, double x0) TwoTwoSum(double a1, double a0, double b1, double b0)
    {
        var (_i, x0) = TwoSum(a0, b0);
        var (_j, _0) = TwoSum(a1, _i);
        var (_i2, x1) = TwoSum(_0, b1);
        var (x3, x2) = TwoSum(_j, _i2);
        return (x3, x2, x1, x0);
    }

    // fast_expansion_sum_zeroelim: sum two nondecreasing-magnitude expansions,
    // eliminating zero components. Writes into h (must be length >= elen+flen).
    // Returns the resulting length. h cannot alias e or f.
    private static int FastExpansionSumZeroElim(int elen, double[] e, int flen, double[] f, double[] h)
    {
        double Q, Qnew, hh;
        int eindex = 0, findex = 0, hindex = 0;
        double enow = e[0];
        double fnow = f[0];
        // (fnow > enow) == (fnow > -enow) is true iff |enow| > |fnow|.
        if ((fnow > enow) == (fnow > -enow))
        {
            Q = enow;
            enow = e[++eindex];
        }
        else
        {
            Q = fnow;
            fnow = f[++findex];
        }
        hindex = 0;
        if (eindex < elen && findex < flen)
        {
            if ((fnow > enow) == (fnow > -enow))
            {
                (Qnew, hh) = FastTwoSum(enow, Q);
                enow = e[++eindex];
            }
            else
            {
                (Qnew, hh) = FastTwoSum(fnow, Q);
                fnow = f[++findex];
            }
            Q = Qnew;
            if (hh != 0.0)
                h[hindex++] = hh;
            while (eindex < elen && findex < flen)
            {
                if ((fnow > enow) == (fnow > -enow))
                {
                    (Qnew, hh) = TwoSum(Q, enow);
                    enow = e[++eindex];
                }
                else
                {
                    (Qnew, hh) = TwoSum(Q, fnow);
                    fnow = f[++findex];
                }
                Q = Qnew;
                if (hh != 0.0)
                    h[hindex++] = hh;
            }
        }
        while (eindex < elen)
        {
            (Qnew, hh) = TwoSum(Q, enow);
            enow = e[++eindex];
            Q = Qnew;
            if (hh != 0.0)
                h[hindex++] = hh;
        }
        while (findex < flen)
        {
            (Qnew, hh) = TwoSum(Q, fnow);
            fnow = f[++findex];
            Q = Qnew;
            if (hh != 0.0)
                h[hindex++] = hh;
        }
        if (Q != 0.0 || hindex == 0)
            h[hindex++] = Q;
        return hindex;
    }

    // scale_expansion_zeroelim: multiply expansion e by scalar b. Writes into h
    // (length >= 2*elen). Returns the resulting length. e and h cannot alias.
    private static int ScaleExpansionZeroElim(int elen, double[] e, double b, double[] h)
    {
        double Q, sum, hh, product1;
        int eindex, hindex;
        double enow;

        Split(b, out var bhi, out var blo);
        // Two_Product_Presplit(e[0], b, bhi, blo, Q, hh):
        double product0;
        product1 = e[0] * b;
        Split(e[0], out var ahi, out var alo);
        double err1 = product1 - (ahi * bhi);
        double err2 = err1 - (alo * bhi);
        double err3 = err2 - (ahi * blo);
        hh = (alo * blo) - err3;
        Q = product1;
        hindex = 0;
        if (hh != 0.0)
            h[hindex++] = hh;
        for (eindex = 1; eindex < elen; eindex++)
        {
            enow = e[eindex];
            // Two_Product_Presplit(enow, b, bhi, blo, product1, product0):
            product1 = enow * b;
            Split(enow, out ahi, out alo);
            err1 = product1 - (ahi * bhi);
            err2 = err1 - (alo * bhi);
            err3 = err2 - (ahi * blo);
            product0 = (alo * blo) - err3;
            (sum, hh) = TwoSum(Q, product0);
            if (hh != 0.0)
                h[hindex++] = hh;
            (Q, hh) = FastTwoSum(product1, sum);
            if (hh != 0.0)
                h[hindex++] = hh;
        }
        if (Q != 0.0 || hindex == 0)
            h[hindex++] = Q;
        return hindex;
    }

    private static double Estimate(int elen, double[] e)
    {
        double Q = 0.0;
        for (int i = 0; i < elen; i++)
            Q += e[i];
        return Q;
    }

    // orient2dadapt: adaptive, exact-sign Orient2D. detsum is the
    // non-cancellation bound of the leading determinant.
    private static double Orient2DAdapt(double ax, double ay, double bx, double by, double cx, double cy, double detsum)
    {
        double acx = ax - cx, bcx = bx - cx, acy = ay - cy, bcy = by - cy;
        double acxtail, acytail, bcxtail, bcytail;

        var (detleft, detlefttail) = TwoProduct(acx, bcy);
        var (detright, detrighttail) = TwoProduct(acy, bcx);

        var (B3, B2, B1, B0) = TwoTwoDiff(detleft, detlefttail, detright, detrighttail);
        double[] B = { B0, B1, B2, B3 };

        double det = Estimate(4, B);
        double errbound = CcwErrBoundB * detsum;
        if (det >= errbound || -det >= errbound)
            return det;

        acxtail = TwoDiffTail(ax, cx, acx);
        bcxtail = TwoDiffTail(bx, cx, bcx);
        acytail = TwoDiffTail(ay, cy, acy);
        bcytail = TwoDiffTail(by, cy, bcy);

        if (acxtail == 0.0 && acytail == 0.0 && bcxtail == 0.0 && bcytail == 0.0)
            return det;

        errbound = CcwErrBoundC * detsum + ResultErrBound * Absolute(det);
        det += (acx * bcytail + bcy * acxtail) - (acy * bcxtail + bcx * acytail);
        if (det >= errbound || -det >= errbound)
            return det;

        var (s1, s0) = TwoProduct(acxtail, bcy);
        var (t1, t0) = TwoProduct(acytail, bcx);
        var (u3, u2, u1, u0) = TwoTwoDiff(s1, s0, t1, t0);
        double[] u = { u0, u1, u2, u3 };
        double[] C1 = new double[8];
        int C1length = FastExpansionSumZeroElim(4, B, 4, u, C1);

        (s1, s0) = TwoProduct(acx, bcytail);
        (t1, t0) = TwoProduct(acy, bcxtail);
        (u3, u2, u1, u0) = TwoTwoDiff(s1, s0, t1, t0);
        u[0] = u0; u[1] = u1; u[2] = u2; u[3] = u3;
        double[] C2 = new double[12];
        int C2length = FastExpansionSumZeroElim(C1length, C1, 4, u, C2);

        (s1, s0) = TwoProduct(acxtail, bcytail);
        (t1, t0) = TwoProduct(acytail, bcxtail);
        (u3, u2, u1, u0) = TwoTwoDiff(s1, s0, t1, t0);
        u[0] = u0; u[1] = u1; u[2] = u2; u[3] = u3;
        double[] D = new double[16];
        int Dlength = FastExpansionSumZeroElim(C2length, C2, 4, u, D);

        return D[Dlength - 1];
    }

    // incircleadapt: adaptive, exact-sign InCircle. permanent is the
    // non-cancellation bound of the leading determinant.
    private static double InCircleAdapt(double ax, double ay, double bx, double by, double cx, double cy, double dx, double dy, double permanent)
    {
        double adx = ax - dx, bdx = bx - dx, cdx = cx - dx;
        double ady = ay - dy, bdy = by - dy, cdy = cy - dy;

        double det, errbound;

        double[] bc = new double[4], ca = new double[4], ab = new double[4];
        double[] axbc = new double[8], axxbc = new double[16], aybc = new double[8], ayybc = new double[16], adet = new double[32];
        int axbclen = 0, axxbclen = 0, aybclen = 0, ayybclen = 0, alen = 0;
        double[] bxca = new double[8], bxxca = new double[16], byca = new double[8], byyca = new double[16], bdet = new double[32];
        int bxcalen = 0, bxxcalen = 0, bycalen = 0, byycalen = 0, blen = 0;
        double[] cxab = new double[8], cxxab = new double[16], cyab = new double[8], cyyab = new double[16], cdet = new double[32];
        int cxablen = 0, cxxablen = 0, cyablen = 0, cyyablen = 0, clen = 0;
        double[] abdet = new double[64];
        int ablen = 0;
        double[] fin1 = new double[1152], fin2 = new double[1152];
        double[] finnow = null!, finother = null!, finswap;
        int finlength = 0;

        double adxtail, bdxtail, cdxtail, adytail, bdytail, cdytail;
        double[] aa = new double[4], bb = new double[4], cc = new double[4];
        double[] u = new double[4], v = new double[4];
        double[] temp8 = new double[8], temp16a = new double[16], temp16b = new double[16], temp16c = new double[16];
        double[] temp32a = new double[32], temp32b = new double[32], temp48 = new double[48], temp64 = new double[64];
        int temp8len = 0, temp16alen = 0, temp16blen = 0, temp16clen = 0;
        int temp32alen = 0, temp32blen = 0, temp48len = 0, temp64len = 0;
        double[] axtbb = new double[8], axtcc = new double[8], aytbb = new double[8], aytcc = new double[8];
        int axtbblen = 0, axtcclen = 0, aytbblen = 0, aytcclen = 0;
        double[] bxtaa = new double[8], bxtcc = new double[8], bytaa = new double[8], bytcc = new double[8];
        int bxtaalen = 0, bxtcclen = 0, bytaalen = 0, bytcclen = 0;
        double[] cxtaa = new double[8], cxtbb = new double[8], cytaa = new double[8], cytbb = new double[8];
        int cxtaalen = 0, cxtbblen = 0, cytaalen = 0, cytbblen = 0;
        double[] axtbc = new double[8], aytbc = new double[8], bxtca = new double[8], bytca = new double[8], cxtab = new double[8], cytab = new double[8];
        int axtbclen = 0, aytbclen = 0, bxtcalen = 0, bytcalen = 0, cxtablen = 0, cytablen = 0;
        double[] axtbct = new double[16], aytbct = new double[16], bxtcat = new double[16], bytcat = new double[16], cxtabt = new double[16], cytabt = new double[16];
        int axtbctlen = 0, aytbctlen = 0, bxtcatlen = 0, bytcatlen = 0, cxtabtlen = 0, cytabtlen = 0;
        double[] axtbctt = new double[8], aytbctt = new double[8], bxtcatt = new double[8];
        double[] bytcatt = new double[8], cxtabtt = new double[8], cytabtt = new double[8];
        int axtbcttlen = 0, aytbcttlen = 0, bxtcattlen = 0, bytcattlen = 0, cxtabttlen = 0, cytabttlen = 0;
        double[] abt = new double[8], bct = new double[8], cat = new double[8];
        int abtlen = 0, bctlen = 0, catlen = 0;
        double[] abtt = new double[4], bctt = new double[4], catt = new double[4];
        int abttlen = 0, bcttlen = 0, cattlen = 0;
        double negate;

        var (bdxcdy1, bdxcdy0) = TwoProduct(bdx, cdy);
        var (cdxbdy1, cdxbdy0) = TwoProduct(cdx, bdy);
        var (bc3, bc2, bc1, bc0) = TwoTwoDiff(bdxcdy1, bdxcdy0, cdxbdy1, cdxbdy0);
        bc[0] = bc0; bc[1] = bc1; bc[2] = bc2; bc[3] = bc3;
        axbclen = ScaleExpansionZeroElim(4, bc, adx, axbc);
        axxbclen = ScaleExpansionZeroElim(axbclen, axbc, adx, axxbc);
        aybclen = ScaleExpansionZeroElim(4, bc, ady, aybc);
        ayybclen = ScaleExpansionZeroElim(aybclen, aybc, ady, ayybc);
        alen = FastExpansionSumZeroElim(axxbclen, axxbc, ayybclen, ayybc, adet);

        var (cdxady1, cdxady0) = TwoProduct(cdx, ady);
        var (adxcdy1, adxcdy0) = TwoProduct(adx, cdy);
        var (ca3, ca2, ca1, ca0) = TwoTwoDiff(cdxady1, cdxady0, adxcdy1, adxcdy0);
        ca[0] = ca0; ca[1] = ca1; ca[2] = ca2; ca[3] = ca3;
        bxcalen = ScaleExpansionZeroElim(4, ca, bdx, bxca);
        bxxcalen = ScaleExpansionZeroElim(bxcalen, bxca, bdx, bxxca);
        bycalen = ScaleExpansionZeroElim(4, ca, bdy, byca);
        byycalen = ScaleExpansionZeroElim(bycalen, byca, bdy, byyca);
        blen = FastExpansionSumZeroElim(bxxcalen, bxxca, byycalen, byyca, bdet);

        var (adxbdy1, adxbdy0) = TwoProduct(adx, bdy);
        var (bdxady1, bdxady0) = TwoProduct(bdx, ady);
        var (ab3, ab2, ab1, ab0) = TwoTwoDiff(adxbdy1, adxbdy0, bdxady1, bdxady0);
        ab[0] = ab0; ab[1] = ab1; ab[2] = ab2; ab[3] = ab3;
        cxablen = ScaleExpansionZeroElim(4, ab, cdx, cxab);
        cxxablen = ScaleExpansionZeroElim(cxablen, cxab, cdx, cxxab);
        cyablen = ScaleExpansionZeroElim(4, ab, cdy, cyab);
        cyyablen = ScaleExpansionZeroElim(cyablen, cyab, cdy, cyyab);
        clen = FastExpansionSumZeroElim(cxxablen, cxxab, cyyablen, cyyab, cdet);

        ablen = FastExpansionSumZeroElim(alen, adet, blen, bdet, abdet);
        finlength = FastExpansionSumZeroElim(ablen, abdet, clen, cdet, fin1);

        det = Estimate(finlength, fin1);
        errbound = IccErrBoundB * permanent;
        if (det >= errbound || -det >= errbound)
            return det;

        adxtail = TwoDiffTail(ax, dx, adx);
        adytail = TwoDiffTail(ay, dy, ady);
        bdxtail = TwoDiffTail(bx, dx, bdx);
        bdytail = TwoDiffTail(by, dy, bdy);
        cdxtail = TwoDiffTail(cx, dx, cdx);
        cdytail = TwoDiffTail(cy, dy, cdy);
        if (adxtail == 0.0 && bdxtail == 0.0 && cdxtail == 0.0
            && adytail == 0.0 && bdytail == 0.0 && cdytail == 0.0)
            return det;

        errbound = IccErrBoundC * permanent + ResultErrBound * Absolute(det);
        det += ((adx * adx + ady * ady) * ((bdx * cdytail + cdy * bdxtail) - (bdy * cdxtail + cdx * bdytail))
                + 2.0 * (adx * adxtail + ady * adytail) * (bdx * cdy - bdy * cdx))
             + ((bdx * bdx + bdy * bdy) * ((cdx * adytail + ady * cdxtail) - (cdy * adxtail + adx * cdytail))
                + 2.0 * (bdx * bdxtail + bdy * bdytail) * (cdx * ady - cdy * adx))
             + ((cdx * cdx + cdy * cdy) * ((adx * bdytail + bdy * adxtail) - (ady * bdxtail + bdx * adytail))
                + 2.0 * (cdx * cdxtail + cdy * cdytail) * (adx * bdy - ady * bdx));
        if (det >= errbound || -det >= errbound)
            return det;

        finnow = fin1;
        finother = fin2;

        if (bdxtail != 0.0 || bdytail != 0.0 || cdxtail != 0.0 || cdytail != 0.0)
        {
            var (adxadx1, adxadx0) = Square(adx);
            var (adyady1, adyady0) = Square(ady);
            var (aa3, aa2, aa1, aa0) = TwoTwoSum(adxadx1, adxadx0, adyady1, adyady0);
            aa[0] = aa0; aa[1] = aa1; aa[2] = aa2; aa[3] = aa3;
        }
        if (cdxtail != 0.0 || cdytail != 0.0 || adxtail != 0.0 || adytail != 0.0)
        {
            var (bdxbdx1, bdxbdx0) = Square(bdx);
            var (bdybdy1, bdybdy0) = Square(bdy);
            var (bb3, bb2, bb1, bb0) = TwoTwoSum(bdxbdx1, bdxbdx0, bdybdy1, bdybdy0);
            bb[0] = bb0; bb[1] = bb1; bb[2] = bb2; bb[3] = bb3;
        }
        if (adxtail != 0.0 || adytail != 0.0 || bdxtail != 0.0 || bdytail != 0.0)
        {
            var (cdxcdx1, cdxcdx0) = Square(cdx);
            var (cdycdy1, cdycdy0) = Square(cdy);
            var (cc3, cc2, cc1, cc0) = TwoTwoSum(cdxcdx1, cdxcdx0, cdycdy1, cdycdy0);
            cc[0] = cc0; cc[1] = cc1; cc[2] = cc2; cc[3] = cc3;
        }

        if (adxtail != 0.0)
        {
            axtbclen = ScaleExpansionZeroElim(4, bc, adxtail, axtbc);
            temp16alen = ScaleExpansionZeroElim(axtbclen, axtbc, 2.0 * adx, temp16a);
            axtcclen = ScaleExpansionZeroElim(4, cc, adxtail, axtcc);
            temp16blen = ScaleExpansionZeroElim(axtcclen, axtcc, bdy, temp16b);
            axtbblen = ScaleExpansionZeroElim(4, bb, adxtail, axtbb);
            temp16clen = ScaleExpansionZeroElim(axtbblen, axtbb, -cdy, temp16c);
            temp32alen = FastExpansionSumZeroElim(temp16alen, temp16a, temp16blen, temp16b, temp32a);
            temp48len = FastExpansionSumZeroElim(temp16clen, temp16c, temp32alen, temp32a, temp48);
            finlength = FastExpansionSumZeroElim(finlength, finnow, temp48len, temp48, finother);
            finswap = finnow; finnow = finother; finother = finswap;
        }
        if (adytail != 0.0)
        {
            aytbclen = ScaleExpansionZeroElim(4, bc, adytail, aytbc);
            temp16alen = ScaleExpansionZeroElim(aytbclen, aytbc, 2.0 * ady, temp16a);
            aytbblen = ScaleExpansionZeroElim(4, bb, adytail, aytbb);
            temp16blen = ScaleExpansionZeroElim(aytbblen, aytbb, cdx, temp16b);
            aytcclen = ScaleExpansionZeroElim(4, cc, adytail, aytcc);
            temp16clen = ScaleExpansionZeroElim(aytcclen, aytcc, -bdx, temp16c);
            temp32alen = FastExpansionSumZeroElim(temp16alen, temp16a, temp16blen, temp16b, temp32a);
            temp48len = FastExpansionSumZeroElim(temp16clen, temp16c, temp32alen, temp32a, temp48);
            finlength = FastExpansionSumZeroElim(finlength, finnow, temp48len, temp48, finother);
            finswap = finnow; finnow = finother; finother = finswap;
        }
        if (bdxtail != 0.0)
        {
            bxtcalen = ScaleExpansionZeroElim(4, ca, bdxtail, bxtca);
            temp16alen = ScaleExpansionZeroElim(bxtcalen, bxtca, 2.0 * bdx, temp16a);
            bxtaalen = ScaleExpansionZeroElim(4, aa, bdxtail, bxtaa);
            temp16blen = ScaleExpansionZeroElim(bxtaalen, bxtaa, cdy, temp16b);
            bxtcclen = ScaleExpansionZeroElim(4, cc, bdxtail, bxtcc);
            temp16clen = ScaleExpansionZeroElim(bxtcclen, bxtcc, -ady, temp16c);
            temp32alen = FastExpansionSumZeroElim(temp16alen, temp16a, temp16blen, temp16b, temp32a);
            temp48len = FastExpansionSumZeroElim(temp16clen, temp16c, temp32alen, temp32a, temp48);
            finlength = FastExpansionSumZeroElim(finlength, finnow, temp48len, temp48, finother);
            finswap = finnow; finnow = finother; finother = finswap;
        }
        if (bdytail != 0.0)
        {
            bytcalen = ScaleExpansionZeroElim(4, ca, bdytail, bytca);
            temp16alen = ScaleExpansionZeroElim(bytcalen, bytca, 2.0 * bdy, temp16a);
            bytcclen = ScaleExpansionZeroElim(4, cc, bdytail, bytcc);
            temp16blen = ScaleExpansionZeroElim(bytcclen, bytcc, adx, temp16b);
            bytaalen = ScaleExpansionZeroElim(4, aa, bdytail, bytaa);
            temp16clen = ScaleExpansionZeroElim(bytaalen, bytaa, -cdx, temp16c);
            temp32alen = FastExpansionSumZeroElim(temp16alen, temp16a, temp16blen, temp16b, temp32a);
            temp48len = FastExpansionSumZeroElim(temp16clen, temp16c, temp32alen, temp32a, temp48);
            finlength = FastExpansionSumZeroElim(finlength, finnow, temp48len, temp48, finother);
            finswap = finnow; finnow = finother; finother = finswap;
        }
        if (cdxtail != 0.0)
        {
            cxtablen = ScaleExpansionZeroElim(4, ab, cdxtail, cxtab);
            temp16alen = ScaleExpansionZeroElim(cxtablen, cxtab, 2.0 * cdx, temp16a);
            cxtbblen = ScaleExpansionZeroElim(4, bb, cdxtail, cxtbb);
            temp16blen = ScaleExpansionZeroElim(cxtbblen, cxtbb, ady, temp16b);
            cxtaalen = ScaleExpansionZeroElim(4, aa, cdxtail, cxtaa);
            temp16clen = ScaleExpansionZeroElim(cxtaalen, cxtaa, -bdy, temp16c);
            temp32alen = FastExpansionSumZeroElim(temp16alen, temp16a, temp16blen, temp16b, temp32a);
            temp48len = FastExpansionSumZeroElim(temp16clen, temp16c, temp32alen, temp32a, temp48);
            finlength = FastExpansionSumZeroElim(finlength, finnow, temp48len, temp48, finother);
            finswap = finnow; finnow = finother; finother = finswap;
        }
        if (cdytail != 0.0)
        {
            cytablen = ScaleExpansionZeroElim(4, ab, cdytail, cytab);
            temp16alen = ScaleExpansionZeroElim(cytablen, cytab, 2.0 * cdy, temp16a);
            cytaalen = ScaleExpansionZeroElim(4, aa, cdytail, cytaa);
            temp16blen = ScaleExpansionZeroElim(cytaalen, cytaa, bdx, temp16b);
            cytbblen = ScaleExpansionZeroElim(4, bb, cdytail, cytbb);
            temp16clen = ScaleExpansionZeroElim(cytbblen, cytbb, -adx, temp16c);
            temp32alen = FastExpansionSumZeroElim(temp16alen, temp16a, temp16blen, temp16b, temp32a);
            temp48len = FastExpansionSumZeroElim(temp16clen, temp16c, temp32alen, temp32a, temp48);
            finlength = FastExpansionSumZeroElim(finlength, finnow, temp48len, temp48, finother);
            finswap = finnow; finnow = finother; finother = finswap;
        }

        if (adxtail != 0.0 || adytail != 0.0)
        {
            if (bdxtail != 0.0 || bdytail != 0.0 || cdxtail != 0.0 || cdytail != 0.0)
            {
                var (ti1, ti0) = TwoProduct(bdxtail, cdy);
                var (tj1, tj0) = TwoProduct(bdx, cdytail);
                var (u3, u2, u1, u0) = TwoTwoSum(ti1, ti0, tj1, tj0);
                u[0] = u0; u[1] = u1; u[2] = u2; u[3] = u3;
                negate = -bdy;
                (ti1, ti0) = TwoProduct(cdxtail, negate);
                negate = -bdytail;
                (tj1, tj0) = TwoProduct(cdx, negate);
                var (v3, v2, v1, v0) = TwoTwoSum(ti1, ti0, tj1, tj0);
                v[0] = v0; v[1] = v1; v[2] = v2; v[3] = v3;
                bctlen = FastExpansionSumZeroElim(4, u, 4, v, bct);

                (ti1, ti0) = TwoProduct(bdxtail, cdytail);
                (tj1, tj0) = TwoProduct(cdxtail, bdytail);
                (v3, v2, v1, v0) = TwoTwoDiff(ti1, ti0, tj1, tj0);
                bctt[0] = v0; bctt[1] = v1; bctt[2] = v2; bctt[3] = v3;
                bcttlen = 4;
            }
            else
            {
                bct[0] = 0.0; bctlen = 1;
                bctt[0] = 0.0; bcttlen = 1;
            }

            if (adxtail != 0.0)
            {
                temp16alen = ScaleExpansionZeroElim(axtbclen, axtbc, adxtail, temp16a);
                axtbctlen = ScaleExpansionZeroElim(bctlen, bct, adxtail, axtbct);
                temp32alen = ScaleExpansionZeroElim(axtbctlen, axtbct, 2.0 * adx, temp32a);
                temp48len = FastExpansionSumZeroElim(temp16alen, temp16a, temp32alen, temp32a, temp48);
                finlength = FastExpansionSumZeroElim(finlength, finnow, temp48len, temp48, finother);
                finswap = finnow; finnow = finother; finother = finswap;
                if (bdytail != 0.0)
                {
                    temp8len = ScaleExpansionZeroElim(4, cc, adxtail, temp8);
                    temp16alen = ScaleExpansionZeroElim(temp8len, temp8, bdytail, temp16a);
                    finlength = FastExpansionSumZeroElim(finlength, finnow, temp16alen, temp16a, finother);
                    finswap = finnow; finnow = finother; finother = finswap;
                }
                if (cdytail != 0.0)
                {
                    temp8len = ScaleExpansionZeroElim(4, bb, -adxtail, temp8);
                    temp16alen = ScaleExpansionZeroElim(temp8len, temp8, cdytail, temp16a);
                    finlength = FastExpansionSumZeroElim(finlength, finnow, temp16alen, temp16a, finother);
                    finswap = finnow; finnow = finother; finother = finswap;
                }

                temp32alen = ScaleExpansionZeroElim(axtbctlen, axtbct, adxtail, temp32a);
                axtbcttlen = ScaleExpansionZeroElim(bcttlen, bctt, adxtail, axtbctt);
                temp16alen = ScaleExpansionZeroElim(axtbcttlen, axtbctt, 2.0 * adx, temp16a);
                temp16blen = ScaleExpansionZeroElim(axtbcttlen, axtbctt, adxtail, temp16b);
                temp32blen = FastExpansionSumZeroElim(temp16alen, temp16a, temp16blen, temp16b, temp32b);
                temp64len = FastExpansionSumZeroElim(temp32alen, temp32a, temp32blen, temp32b, temp64);
                finlength = FastExpansionSumZeroElim(finlength, finnow, temp64len, temp64, finother);
                finswap = finnow; finnow = finother; finother = finswap;
            }
            if (adytail != 0.0)
            {
                temp16alen = ScaleExpansionZeroElim(aytbclen, aytbc, adytail, temp16a);
                aytbctlen = ScaleExpansionZeroElim(bctlen, bct, adytail, aytbct);
                temp32alen = ScaleExpansionZeroElim(aytbctlen, aytbct, 2.0 * ady, temp32a);
                temp48len = FastExpansionSumZeroElim(temp16alen, temp16a, temp32alen, temp32a, temp48);
                finlength = FastExpansionSumZeroElim(finlength, finnow, temp48len, temp48, finother);
                finswap = finnow; finnow = finother; finother = finswap;

                temp32alen = ScaleExpansionZeroElim(aytbctlen, aytbct, adytail, temp32a);
                aytbcttlen = ScaleExpansionZeroElim(bcttlen, bctt, adytail, aytbctt);
                temp16alen = ScaleExpansionZeroElim(aytbcttlen, aytbctt, 2.0 * ady, temp16a);
                temp16blen = ScaleExpansionZeroElim(aytbcttlen, aytbctt, adytail, temp16b);
                temp32blen = FastExpansionSumZeroElim(temp16alen, temp16a, temp16blen, temp16b, temp32b);
                temp64len = FastExpansionSumZeroElim(temp32alen, temp32a, temp32blen, temp32b, temp64);
                finlength = FastExpansionSumZeroElim(finlength, finnow, temp64len, temp64, finother);
                finswap = finnow; finnow = finother; finother = finswap;
            }
        }
        if (bdxtail != 0.0 || bdytail != 0.0)
        {
            if (cdxtail != 0.0 || cdytail != 0.0 || adxtail != 0.0 || adytail != 0.0)
            {
                var (ti1, ti0) = TwoProduct(cdxtail, ady);
                var (tj1, tj0) = TwoProduct(cdx, adytail);
                var (u3, u2, u1, u0) = TwoTwoSum(ti1, ti0, tj1, tj0);
                u[0] = u0; u[1] = u1; u[2] = u2; u[3] = u3;
                negate = -cdy;
                (ti1, ti0) = TwoProduct(adxtail, negate);
                negate = -cdytail;
                (tj1, tj0) = TwoProduct(adx, negate);
                var (v3, v2, v1, v0) = TwoTwoSum(ti1, ti0, tj1, tj0);
                v[0] = v0; v[1] = v1; v[2] = v2; v[3] = v3;
                catlen = FastExpansionSumZeroElim(4, u, 4, v, cat);

                (ti1, ti0) = TwoProduct(cdxtail, adytail);
                (tj1, tj0) = TwoProduct(adxtail, cdytail);
                (v3, v2, v1, v0) = TwoTwoDiff(ti1, ti0, tj1, tj0);
                catt[0] = v0; catt[1] = v1; catt[2] = v2; catt[3] = v3;
                cattlen = 4;
            }
            else
            {
                cat[0] = 0.0; catlen = 1;
                catt[0] = 0.0; cattlen = 1;
            }

            if (bdxtail != 0.0)
            {
                temp16alen = ScaleExpansionZeroElim(bxtcalen, bxtca, bdxtail, temp16a);
                bxtcatlen = ScaleExpansionZeroElim(catlen, cat, bdxtail, bxtcat);
                temp32alen = ScaleExpansionZeroElim(bxtcatlen, bxtcat, 2.0 * bdx, temp32a);
                temp48len = FastExpansionSumZeroElim(temp16alen, temp16a, temp32alen, temp32a, temp48);
                finlength = FastExpansionSumZeroElim(finlength, finnow, temp48len, temp48, finother);
                finswap = finnow; finnow = finother; finother = finswap;
                if (cdytail != 0.0)
                {
                    temp8len = ScaleExpansionZeroElim(4, aa, bdxtail, temp8);
                    temp16alen = ScaleExpansionZeroElim(temp8len, temp8, cdytail, temp16a);
                    finlength = FastExpansionSumZeroElim(finlength, finnow, temp16alen, temp16a, finother);
                    finswap = finnow; finnow = finother; finother = finswap;
                }
                if (adytail != 0.0)
                {
                    temp8len = ScaleExpansionZeroElim(4, cc, -bdxtail, temp8);
                    temp16alen = ScaleExpansionZeroElim(temp8len, temp8, adytail, temp16a);
                    finlength = FastExpansionSumZeroElim(finlength, finnow, temp16alen, temp16a, finother);
                    finswap = finnow; finnow = finother; finother = finswap;
                }

                temp32alen = ScaleExpansionZeroElim(bxtcatlen, bxtcat, bdxtail, temp32a);
                bxtcattlen = ScaleExpansionZeroElim(cattlen, catt, bdxtail, bxtcatt);
                temp16alen = ScaleExpansionZeroElim(bxtcattlen, bxtcatt, 2.0 * bdx, temp16a);
                temp16blen = ScaleExpansionZeroElim(bxtcattlen, bxtcatt, bdxtail, temp16b);
                temp32blen = FastExpansionSumZeroElim(temp16alen, temp16a, temp16blen, temp16b, temp32b);
                temp64len = FastExpansionSumZeroElim(temp32alen, temp32a, temp32blen, temp32b, temp64);
                finlength = FastExpansionSumZeroElim(finlength, finnow, temp64len, temp64, finother);
                finswap = finnow; finnow = finother; finother = finswap;
            }
            if (bdytail != 0.0)
            {
                temp16alen = ScaleExpansionZeroElim(bytcalen, bytca, bdytail, temp16a);
                bytcatlen = ScaleExpansionZeroElim(catlen, cat, bdytail, bytcat);
                temp32alen = ScaleExpansionZeroElim(bytcatlen, bytcat, 2.0 * bdy, temp32a);
                temp48len = FastExpansionSumZeroElim(temp16alen, temp16a, temp32alen, temp32a, temp48);
                finlength = FastExpansionSumZeroElim(finlength, finnow, temp48len, temp48, finother);
                finswap = finnow; finnow = finother; finother = finswap;

                temp32alen = ScaleExpansionZeroElim(bytcatlen, bytcat, bdytail, temp32a);
                bytcattlen = ScaleExpansionZeroElim(cattlen, catt, bdytail, bytcatt);
                temp16alen = ScaleExpansionZeroElim(bytcattlen, bytcatt, 2.0 * bdy, temp16a);
                temp16blen = ScaleExpansionZeroElim(bytcattlen, bytcatt, bdytail, temp16b);
                temp32blen = FastExpansionSumZeroElim(temp16alen, temp16a, temp16blen, temp16b, temp32b);
                temp64len = FastExpansionSumZeroElim(temp32alen, temp32a, temp32blen, temp32b, temp64);
                finlength = FastExpansionSumZeroElim(finlength, finnow, temp64len, temp64, finother);
                finswap = finnow; finnow = finother; finother = finswap;
            }
        }
        if (cdxtail != 0.0 || cdytail != 0.0)
        {
            if (adxtail != 0.0 || adytail != 0.0 || bdxtail != 0.0 || bdytail != 0.0)
            {
                var (ti1, ti0) = TwoProduct(adxtail, bdy);
                var (tj1, tj0) = TwoProduct(adx, bdytail);
                var (u3, u2, u1, u0) = TwoTwoSum(ti1, ti0, tj1, tj0);
                u[0] = u0; u[1] = u1; u[2] = u2; u[3] = u3;
                negate = -ady;
                (ti1, ti0) = TwoProduct(bdxtail, negate);
                negate = -adytail;
                (tj1, tj0) = TwoProduct(bdx, negate);
                var (v3, v2, v1, v0) = TwoTwoSum(ti1, ti0, tj1, tj0);
                v[0] = v0; v[1] = v1; v[2] = v2; v[3] = v3;
                abtlen = FastExpansionSumZeroElim(4, u, 4, v, abt);

                (ti1, ti0) = TwoProduct(adxtail, bdytail);
                (tj1, tj0) = TwoProduct(bdxtail, adytail);
                (v3, v2, v1, v0) = TwoTwoDiff(ti1, ti0, tj1, tj0);
                abtt[0] = v0; abtt[1] = v1; abtt[2] = v2; abtt[3] = v3;
                abttlen = 4;
            }
            else
            {
                abt[0] = 0.0; abtlen = 1;
                abtt[0] = 0.0; abttlen = 1;
            }

            if (cdxtail != 0.0)
            {
                temp16alen = ScaleExpansionZeroElim(cxtablen, cxtab, cdxtail, temp16a);
                cxtabtlen = ScaleExpansionZeroElim(abtlen, abt, cdxtail, cxtabt);
                temp32alen = ScaleExpansionZeroElim(cxtabtlen, cxtabt, 2.0 * cdx, temp32a);
                temp48len = FastExpansionSumZeroElim(temp16alen, temp16a, temp32alen, temp32a, temp48);
                finlength = FastExpansionSumZeroElim(finlength, finnow, temp48len, temp48, finother);
                finswap = finnow; finnow = finother; finother = finswap;
                if (adytail != 0.0)
                {
                    temp8len = ScaleExpansionZeroElim(4, bb, cdxtail, temp8);
                    temp16alen = ScaleExpansionZeroElim(temp8len, temp8, adytail, temp16a);
                    finlength = FastExpansionSumZeroElim(finlength, finnow, temp16alen, temp16a, finother);
                    finswap = finnow; finnow = finother; finother = finswap;
                }
                if (bdytail != 0.0)
                {
                    temp8len = ScaleExpansionZeroElim(4, aa, -cdxtail, temp8);
                    temp16alen = ScaleExpansionZeroElim(temp8len, temp8, bdytail, temp16a);
                    finlength = FastExpansionSumZeroElim(finlength, finnow, temp16alen, temp16a, finother);
                    finswap = finnow; finnow = finother; finother = finswap;
                }

                temp32alen = ScaleExpansionZeroElim(cxtabtlen, cxtabt, cdxtail, temp32a);
                cxtabttlen = ScaleExpansionZeroElim(abttlen, abtt, cdxtail, cxtabtt);
                temp16alen = ScaleExpansionZeroElim(cxtabttlen, cxtabtt, 2.0 * cdx, temp16a);
                temp16blen = ScaleExpansionZeroElim(cxtabttlen, cxtabtt, cdxtail, temp16b);
                temp32blen = FastExpansionSumZeroElim(temp16alen, temp16a, temp16blen, temp16b, temp32b);
                temp64len = FastExpansionSumZeroElim(temp32alen, temp32a, temp32blen, temp32b, temp64);
                finlength = FastExpansionSumZeroElim(finlength, finnow, temp64len, temp64, finother);
                finswap = finnow; finnow = finother; finother = finswap;
            }
            if (cdytail != 0.0)
            {
                temp16alen = ScaleExpansionZeroElim(cytablen, cytab, cdytail, temp16a);
                cytabtlen = ScaleExpansionZeroElim(abtlen, abt, cdytail, cytabt);
                temp32alen = ScaleExpansionZeroElim(cytabtlen, cytabt, 2.0 * cdy, temp32a);
                temp48len = FastExpansionSumZeroElim(temp16alen, temp16a, temp32alen, temp32a, temp48);
                finlength = FastExpansionSumZeroElim(finlength, finnow, temp48len, temp48, finother);
                finswap = finnow; finnow = finother; finother = finswap;

                temp32alen = ScaleExpansionZeroElim(cytabtlen, cytabt, cdytail, temp32a);
                cytabttlen = ScaleExpansionZeroElim(abttlen, abtt, cdytail, cytabtt);
                temp16alen = ScaleExpansionZeroElim(cytabttlen, cytabtt, 2.0 * cdy, temp16a);
                temp16blen = ScaleExpansionZeroElim(cytabttlen, cytabtt, cdytail, temp16b);
                temp32blen = FastExpansionSumZeroElim(temp16alen, temp16a, temp16blen, temp16b, temp32b);
                temp64len = FastExpansionSumZeroElim(temp32alen, temp32a, temp32blen, temp32b, temp64);
                finlength = FastExpansionSumZeroElim(finlength, finnow, temp64len, temp64, finother);
                finswap = finnow; finnow = finother; finother = finswap;
            }
        }

        return finnow[finlength - 1];
    }

    // Public adaptive predicates.

    // Returns a positive value if C is strictly to the left of the directed line AB
    // (i.e. ABC is counterclockwise), negative if to the right, and zero if collinear.
    // The sign of the return value is exact; the magnitude is the determinant.
    public static double Orient2D(double ax, double ay, double bx, double by, double cx, double cy)
    {
        double detleft = (ax - cx) * (by - cy);
        double detright = (ay - cy) * (bx - cx);
        double det = detleft - detright;

        double detsum;
        if (detleft > 0.0)
        {
            if (detright <= 0.0)
                return det;
            detsum = detleft + detright;
        }
        else if (detleft < 0.0)
        {
            if (detright >= 0.0)
                return det;
            detsum = -detleft - detright;
        }
        else
        {
            return det;
        }

        double errbound = CcwErrBoundA * detsum;
        if (det >= errbound || -det >= errbound)
            return det;
        return Orient2DAdapt(ax, ay, bx, by, cx, cy, detsum);
    }

    public static double Orient2D(System.Numerics.Vector2 a, System.Numerics.Vector2 b, System.Numerics.Vector2 c)
        => Orient2D(a.X, a.Y, b.X, b.Y, c.X, c.Y);

    // Returns a positive value if D is strictly inside the circumcircle of ABC
    // (assuming ABC is counterclockwise), negative if outside, zero if cocircular.
    // The sign is exact.
    public static double InCircle(double ax, double ay, double bx, double by, double cx, double cy, double dx, double dy)
    {
        double adx = ax - dx, bdx = bx - dx, cdx = cx - dx;
        double ady = ay - dy, bdy = by - dy, cdy = cy - dy;

        double bdxcdy = bdx * cdy;
        double cdxbdy = cdx * bdy;
        double alift = adx * adx + ady * ady;

        double cdxady = cdx * ady;
        double adxcdy = adx * cdy;
        double blift = bdx * bdx + bdy * bdy;

        double adxbdy = adx * bdy;
        double bdxady = bdx * ady;
        double clift = cdx * cdx + cdy * cdy;

        double det = alift * (bdxcdy - cdxbdy) + blift * (cdxady - adxcdy) + clift * (adxbdy - bdxady);

        double permanent = (Absolute(bdxcdy) + Absolute(cdxbdy)) * alift
                         + (Absolute(cdxady) + Absolute(adxcdy)) * blift
                         + (Absolute(adxbdy) + Absolute(bdxady)) * clift;
        double errbound = IccErrBoundA * permanent;
        if (det > errbound || -det > errbound)
            return det;
        return InCircleAdapt(ax, ay, bx, by, cx, cy, dx, dy, permanent);
    }

    public static double InCircle(System.Numerics.Vector2 a, System.Numerics.Vector2 b, System.Numerics.Vector2 c, System.Numerics.Vector2 d)
        => InCircle(a.X, a.Y, b.X, b.Y, c.X, c.Y, d.X, d.Y);
}