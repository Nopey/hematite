OUTPUT = block.vert.spv block.frag.spv

help:
	#
	# This makefile builds the shaders into spir-v using glslc
	# To use, run `make spv`
	#

spv: ${OUTPUT}
	

%.spv: %
	glslc -o $@ $^
	# tell rustc to rebuild with new spv files
	touch shader.rs

clean:
	rm ${OUTPUT} -f

.PHONY: spv help clean
