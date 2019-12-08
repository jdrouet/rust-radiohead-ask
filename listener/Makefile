all: build rm run

run: rm
	docker run --rm -d \
		--privileged \
		-e READ_SPEED=2000 \
		-e GPIO_PIN=27 \
		--name ask-listener \
		ask-listener:latest

build:
	docker build -t ask-listener:latest .

rm:
	-docker rm -f ask-listener
