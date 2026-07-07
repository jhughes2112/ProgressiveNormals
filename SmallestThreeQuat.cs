using System;
using Quat = System.Numerics.Quaternion;

namespace ProgressiveNormals
{
	// Smallest-three quaternion encoding with Morton-interleaved component bits, so it truncates
	// the same way OctMortonNormal does: encode once at full precision (62 bits) and any prefix
	// (of at least 2 bits) is a valid coarser encoding of the same rotation.
	// Layout (MSB first): 2 bits for which component was largest (never truncated away), then the
	// three remaining components' bits round-robin interleaved: a19, b19, c19, a18, b18, c18, ...
	public static class SmallestThreeQuat
	{
		public const int MaxBits = 62;
		private const int kAxisBits = 20;
		private const float kAxisScale = 1 << kAxisBits;
		private const float kSqrt2 = 1.41421356f;
		private const float kInvSqrt2 = 0.70710678f;

		// Full-precision encode.  Input must be unit length; q and -q encode to the same rotation.
		public static ulong Encode(Quat q)
		{
			Span<float> c = stackalloc float[4] { q.X, q.Y, q.Z, q.W };

			int largest = 0;
			for (int i = 1; i < 4; i++)
				if (MathF.Abs(c[i]) > MathF.Abs(c[largest]))
					largest = i;

			if (c[largest] < 0f)  // force the dropped component positive so it reconstructs unambiguously
				for (int i = 0; i < 4; i++)
					c[i] = -c[i];

			// The three survivors, in component order, each guaranteed within [-1/sqrt2, 1/sqrt2].
			Span<uint> s = stackalloc uint[3];
			int n = 0;
			for (int i = 0; i < 4; i++)
				if (i != largest)
					s[n++] = Quantize(c[i]);

			ulong interleaved = (Part1By2(s[0]) << 2) | (Part1By2(s[1]) << 1) | Part1By2(s[2]);
			return ((ulong)largest << 60) | interleaved;
		}

		// Keeps the top 'bits' bits as the coarse code, right-aligned.  bits must be >= 2 so the
		// largest-component index survives.
		public static ulong Truncate(ulong code, int bits)
		{
			return code >> (MaxBits - bits);
		}

		// Decodes a right-aligned 'bits'-bit code (as produced by Truncate, or Encode when bits == 62).
		public static Quat Decode(ulong code, int bits)
		{
			code <<= (MaxBits - bits);            // restore full bit positions; dropped bits become zero
			int largest = (int)(code >> 60) & 3;
			ulong body = code & 0x0FFFFFFFFFFFFFFF;  // strip the index bits so they can't leak into compaction

			int rem = bits - 2;
			Span<float> s = stackalloc float[3];
			s[0] = Dequantize(Compact1By2(body >> 2), (rem + 2) / 3);  // a leads the round-robin
			s[1] = Dequantize(Compact1By2(body >> 1), (rem + 1) / 3);
			s[2] = Dequantize(Compact1By2(body), rem / 3);

			Span<float> c = stackalloc float[4];
			int n = 0;
			float sumSq = 0f;
			for (int i = 0; i < 4; i++)
			{
				if (i != largest)
				{
					c[i] = s[n++];
					sumSq += c[i] * c[i];
				}
			}
			c[largest] = MathF.Sqrt(MathF.Max(0f, 1f - sumSq));

			return Quat.Normalize(new Quat(c[0], c[1], c[2], c[3]));
		}

		// ---- quantization: [-1/sqrt2, 1/sqrt2] <-> 20-bit cell, floor on encode / cell center on decode ----

		private static uint Quantize(float f)
		{
			float t = f * kInvSqrt2 + 0.5f;  // [0,1]
			return (uint)Math.Clamp((int)(t * kAxisScale), 0, (1 << kAxisBits) - 1);
		}

		private static float Dequantize(ulong q, int significantBits)
		{
			// q has its low (kAxisBits - significantBits) bits zeroed; add half a cell at that level.
			float halfCell = 0.5f * (1u << (kAxisBits - significantBits));
			return ((q + halfCell) / kAxisScale) * kSqrt2 - kInvSqrt2;
		}

		// ---- 3-way Morton bit spreading/compaction (21-bit capable, we use 20) ----

		private static ulong Part1By2(ulong x)  // bit i -> bit 3i
		{
			x &= 0x1FFFFF;
			x = (x | (x << 32)) & 0x001F00000000FFFF;
			x = (x | (x << 16)) & 0x001F0000FF0000FF;
			x = (x | (x << 8))  & 0x100F00F00F00F00F;
			x = (x | (x << 4))  & 0x10C30C30C30C30C3;
			x = (x | (x << 2))  & 0x1249249249249249;
			return x;
		}

		private static ulong Compact1By2(ulong x)  // bit 3i -> bit i
		{
			x &= 0x1249249249249249;
			x = (x | (x >> 2))  & 0x10C30C30C30C30C3;
			x = (x | (x >> 4))  & 0x100F00F00F00F00F;
			x = (x | (x >> 8))  & 0x001F0000FF0000FF;
			x = (x | (x >> 16)) & 0x001F00000000FFFF;
			x = (x | (x >> 32)) & 0x1FFFFF;
			return x;
		}
	}
}
