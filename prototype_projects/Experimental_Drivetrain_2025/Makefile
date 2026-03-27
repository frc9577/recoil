WORKDIR ?= $(shell pwd)

WPILIB_CONTAINER := wpilib/roborio-cross-ubuntu:2025-22.04

.PHONY: build
build:
	bash common-repo-sync.sh
	docker run -it -v ${WORKDIR}:/work -w /work -e GRADLE_USER_HOME=/work/.gradle-home ${WPILIB_CONTAINER} sh -c "./gradlew build"

.PHONY: deploy
deploy:
	docker run -it -v ${WORKDIR}:/work -w /work -e GRADLE_USER_HOME=/work/.gradle-home ${WPILIB_CONTAINER} sh -c "./gradlew deploy"

.PHONY: clean
clean:
	docker run -it -v ${WORKDIR}:/work -w /work -e GRADLE_USER_HOME=/work/.gradle-home ${WPILIB_CONTAINER} sh -c "rm -Rf build"

.PHONY: deep-clean
deep-clean: clean
	docker run -it -v ${WORKDIR}:/work -w /work -e GRADLE_USER_HOME=/work/.gradle-home ${WPILIB_CONTAINER} sh -c "rm -Rf .gradle && rm -Rf .gradle-home"
	rm -Rf src/main/java/frc/recoil
	git clean -fdx