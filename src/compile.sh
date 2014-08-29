if [[ "$1" == "proc" ]]; then
	g++  -g image.cpp imageloader.cpp camera.cpp rectifier.cpp qrcode.cpp  processing.cpp -I /usr/include/eigen3/ -o app -lzbar `pkg-config --cflags --libs opencv` && ./app
elif [[ "$1" == "bench" ]]; then
g++  -g image.cpp imageloader.cpp camera.cpp rectifier.cpp qrcode.cpp  benchmarks.cpp -I /usr/include/eigen3/ -o app -lzbar `pkg-config --cflags --libs opencv` && ./app
elif [[ "$1" == "pro" ]]; then
	g++  -g image.cpp imageloader.cpp camera.cpp rectifier.cpp qrcode.cpp  propro.cpp -I /usr/include/eigen3/ -o app -lzbar `pkg-config --cflags --libs opencv` && ./app
elif [[ "$1" == "rect" ]]; then
	g++  rectify_images.cpp  -o app `pkg-config --cflags --libs opencv` && ./app
elif [[ "$1" == "pos" ]]; then
	g++ image.cpp imageloader.cpp camera.cpp rectifier.cpp qrcode.cpp  pos.cpp -I /usr/include/eigen3/ -o app -lzbar `pkg-config --cflags --libs opencv` && ./app
elif [[ "$1" == "example" ]]; then
	g++ -g image.cpp imageloader.cpp camera.cpp rectifier.cpp qrcode.cpp  lastMain.cpp -I /usr/include/eigen3/  -o example -lzbar `pkg-config --cflags --libs opencv` && ./example
else
	printf "A parameter is required:\nmain: for acquisition\nproc:for processing\n"
fi
