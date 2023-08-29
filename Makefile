build:
	cmake -S ./ -B ./build
	cmake --build build

run: build
	./build/MechatronicArmAnalyzer

clean:
	find ./ -name "build" -type d -exec rm -rf {} +
