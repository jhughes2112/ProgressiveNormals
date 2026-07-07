using System;
using Vec3 = System.Numerics.Vector3;

namespace ProgressiveNormals
{
	// Octahedral-mapped, Morton-interleaved unit vector encoding.
	// Encode once at full precision (32 bits); any prefix of that bit stream is itself a valid
	// coarser encoding of the same vector, so truncation IS the lower-precision encoding.
	// Layout (MSB first): u15, v15, u14, v14, ... u0, v0 — each added bit halves the current cell
	// on alternating axes.
	public static class OctMortonNormal
	{
		public const int MaxBits = 32;
		private const int kAxisBits = 16;
		private const float kAxisScale = 1 << kAxisBits;

		// Full-precision encode.  Input must be unit length (not the zero vector).
		public static uint Encode(Vec3 n)
		{
			(float u, float v) = OctEncode(n);
			return (Part1By1(Quantize(u)) << 1) | Part1By1(Quantize(v));
		}

		// Keeps the top 'bits' bits as the coarse code, right-aligned — this is what you'd put on the wire.
		public static uint Truncate(uint code, int bits)
		{
			return bits <= 0 ? 0u : code >> (MaxBits - bits);
		}

		// Decodes a right-aligned 'bits'-bit code (as produced by Truncate, or Encode when bits == 32).
		// Reconstructs at the center of the cell the surviving bits describe.
		public static Vec3 Decode(uint code, int bits)
		{
			code <<= (MaxBits - bits);            // restore full bit positions; dropped bits become zero
			int uBits = (bits + 1) / 2;           // u owns the leading bit, so it gets the extra one when odd
			int vBits = bits / 2;
			float u = Dequantize(Compact1By1(code >> 1), uBits);
			float v = Dequantize(Compact1By1(code), vBits);
			return OctDecode(u, v);
		}

		// ---- octahedral mapping (Cigolle et al. 2014) ----

		private static (float u, float v) OctEncode(Vec3 n)
		{
			float invL1 = 1f / (MathF.Abs(n.X) + MathF.Abs(n.Y) + MathF.Abs(n.Z));
			float x = n.X * invL1;
			float y = n.Y * invL1;
			if (n.Z < 0f)  // fold the lower hemisphere over the diagonals
			{
				float ox = x;
				x = (1f - MathF.Abs(y)) * SignNotZero(ox);
				y = (1f - MathF.Abs(ox)) * SignNotZero(y);
			}
			return (x, y);
		}

		private static Vec3 OctDecode(float u, float v)
		{
			float z = 1f - MathF.Abs(u) - MathF.Abs(v);
			if (z < 0f)  // unfold
			{
				float ou = u;
				u = (1f - MathF.Abs(v)) * SignNotZero(ou);
				v = (1f - MathF.Abs(ou)) * SignNotZero(v);
			}
			return Vec3.Normalize(new Vec3(u, v, z));
		}

		private static float SignNotZero(float f) => f >= 0f ? 1f : -1f;

		// ---- quantization: [-1,1] <-> 16-bit cell, floor on encode / cell center on decode ----

		private static uint Quantize(float f)
		{
			float t = (f + 1f) * 0.5f;  // [0,1]
			return (uint)Math.Min(kAxisBits == 16 ? 65535 : (1 << kAxisBits) - 1, (int)(t * kAxisScale));
		}

		private static float Dequantize(uint q, int significantBits)
		{
			// q has its low (kAxisBits - significantBits) bits zeroed; add half a cell at that level.
			float halfCell = 0.5f * (1u << (kAxisBits - significantBits));
			return ((q + halfCell) / kAxisScale) * 2f - 1f;
		}

		// ---- 2-way Morton bit spreading/compaction ----

		private static uint Part1By1(uint x)  // bit i -> bit 2i
		{
			x &= 0x0000FFFF;
			x = (x | (x << 8)) & 0x00FF00FF;
			x = (x | (x << 4)) & 0x0F0F0F0F;
			x = (x | (x << 2)) & 0x33333333;
			x = (x | (x << 1)) & 0x55555555;
			return x;
		}

		private static uint Compact1By1(uint x)  // bit 2i -> bit i
		{
			x &= 0x55555555;
			x = (x | (x >> 1)) & 0x33333333;
			x = (x | (x >> 2)) & 0x0F0F0F0F;
			x = (x | (x >> 4)) & 0x00FF00FF;
			x = (x | (x >> 8)) & 0x0000FFFF;
			return x;
		}
	}
}
