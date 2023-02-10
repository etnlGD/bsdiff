/*-
 * Copyright 2003-2005 Colin Percival
 * Copyright 2012 Matthew Endsley
 * All rights reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted providing that the following conditions 
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <limits.h>
#include "bspatch.h"

static int32_t offtin(uint8_t *buf)
{
	int32_t y;

	y=buf[3]&0x7F;
	y=y*256;y+=buf[2];
	y=y*256;y+=buf[1];
	y=y*256;y+=buf[0];

	if(buf[3]&0x80) y=-y;

	return y;
}

int bspatch(const uint8_t* old, int32_t oldsize, uint8_t* new, int32_t newsize, struct bspatch_stream* stream)
{
	uint8_t buf[4];
	int32_t oldpos,newpos;
	int32_t ctrl[3];
	int32_t i;

	oldpos=0;newpos=0;
	while(newpos<newsize) {
		/* Read control data */
		for(i=0;i<=2;i++) {
			if (stream->read(stream, buf, 4))
				return -1;
			ctrl[i]=offtin(buf);
		};

		/* Sanity-check */
		if (ctrl[0]<0 || ctrl[0]>INT_MAX ||
			ctrl[1]<0 || ctrl[1]>INT_MAX ||
			newpos+ctrl[0]>newsize)
			return -1;

		/* Read diff string */
		if (stream->read(stream, new + newpos, ctrl[0]))
			return -1;

		/* Add old data to diff string */
		for(i=0;i<ctrl[0];i++)
			if((oldpos+i>=0) && (oldpos+i<oldsize))
				new[newpos+i]+=old[oldpos+i];

		/* Adjust pointers */
		newpos+=ctrl[0];
		oldpos+=ctrl[0];

		/* Sanity-check */
		if(newpos+ctrl[1]>newsize)
			return -1;

		/* Read extra string */
		if (stream->read(stream, new + newpos, ctrl[1]))
			return -1;

		/* Adjust pointers */
		newpos+=ctrl[1];
		oldpos+=ctrl[2];
	};

	return 0;
}

#include "bsdiff.h"
#if BS_BUILD_BINARY && !BS_DIFF_BINARY

#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

static int offset = 0;
static int bz2_read(const struct bspatch_stream* stream, void* buffer, int length)
{
	uint8_t* diff = (uint8_t*)stream->opaque;
	memcpy(buffer, &diff[offset], length);
	offset += length;

	return 0;
}

static uint8_t* readFile(const char* file, int* pSize)
{
	FILE* f = fopen(file, "rb");
	fseek(f, 0, SEEK_END);
	long size = ftell(f);
	fseek(f, 0, SEEK_SET);

	uint8_t* buf = (uint8_t*)malloc(size);
	if (buf)
		fread(buf, 1, size, f);

	*pSize = size;
	return buf;
}

int main(int argc,char * argv[])
{
	if (argc != 4) {
		printf("usage: %s oldfile newfile patchfile\n", argv[0]);
		return -1;
	}

	int oldsize, diffSize;
	uint8_t* old = readFile(argv[1], &oldsize);
	uint8_t* diff = readFile(argv[3], &diffSize);
	int newsize = ((int*)diff)[0];
	uint8_t* new = malloc(newsize);

	struct bspatch_stream stream;
	stream.read = bz2_read;
	stream.opaque = diff + 4;
	if (bspatch(old, oldsize, new, newsize, &stream))
		return 1;

	/* Clean up the bzip2 reads */
	/* Write the new file */
	FILE* newfile = fopen(argv[2], "wb");
	fwrite(new, 1, newsize, newfile);
	fclose(newfile);

	return 0;
}
#endif
