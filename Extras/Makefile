build-dir := build/esp32.esp32.XIAO_ESP32S3
sketch-name := Compass

.PHONY: compile
compile: ${build-dir}/${sketch-name}.ino.bin

${build-dir}/${sketch-name}.ino.bin: ${sketch-name}.ino
	arduino-cli compile . --config-file ../../arduino.yaml

.PHONY: upload
upload: ${build-dir}/${sketch-name}.ino.bin
	arduino-cli upload . --input-dir ${build-dir}

.PHONY: clean
clean:
	rm -rf build
