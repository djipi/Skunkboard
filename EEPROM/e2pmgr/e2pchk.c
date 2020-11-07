/**********************************************************************
 * Copyright 2020, James Jones
 * SPDX-License-Identifier: CC0-1.0
 **********************************************************************/
#include <stdio.h>
#include <inttypes.h>

#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
#define SWAP_WORD(w) (w) = (((w) << 8) | ((w) >> 8))
#else
#define SWAP_WORD(w) (void)(w)
#endif

int main(int argc, char *argv[])
{
	FILE *f;
	int size = 0;
	uint16_t w, wlast = 0, chksum = 0;

	if (argc < 2 || fopen_s(&f, argv[1], "rb")) {
		printf("usage: e2pchk <file.e2p>");
		return 255;
	}

	/* Sanity-check file size to save user from themselves */
	while (size < 1025) {
		if (fread_s(&w, sizeof(w), sizeof(w), 1, f) != 1) {
			if (feof(f)) {
				break;
			}
			fprintf(stderr, "ERROR: Failed to read EEPROM file\n");
			return 1;
		}
		chksum += wlast;

		SWAP_WORD(w);
		wlast = w;
		size++;
	}

	switch (size) {
	case 64: /* fallthrough */
	case 1024:
		break;
	default:
		fprintf(stderr, "ERROR: Invalid EEPROM file size: %d%s\n",
			size * 2, (size == 1025) ? "+" : "");
		return 2;
	}

	chksum ^= 0xFFFF;

	if (wlast != chksum) {
		fprintf(stderr, "ERROR: Checksum mismatch\n");
		fprintf(stderr, "  Calculated: 0x%04" PRIx16
			" File: 0x%04" PRIx16 "\n", chksum, wlast);
		return 3;
	}

	fclose(f);

	printf("SUCCESS: Checksums match: 0x%04" PRIx16 "\n", chksum);

	return 0;
}
