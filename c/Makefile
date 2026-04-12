all: hello_cpp can1_tx_can2_rx

hello_cpp: main.cpp
	g++ -o hello_cpp main.cpp ./libcontrolcan.so -lpthread

can1_tx_can2_rx: can1_tx_can2_rx.cpp
	g++ -o can1_tx_can2_rx can1_tx_can2_rx.cpp ./libcontrolcan.so -lpthread

clean:
	rm -f *.o hello_cpp can1_tx_can2_rx
