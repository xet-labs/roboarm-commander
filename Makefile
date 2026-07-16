# makefile

APP_NAME ?= roboarm-commander
VERSION ?= v0.1.0

OS := linux windows darwin
ARCH := amd64 arm64

BUILD_DIR := build

mod:
	go mod tidy -v                   
	go mod download
	go mod vendor

check:
	go fmt ./...
	go vet ./...

run:
	@AppBin="${PWD}/storage/tmp/bin"; \
	echo "Creating bin dir at $$AppBin"; \
	mkdir -p "$$AppBin"; \
	\
	if [ ! -f "$$AppBin/air" ]; then \
		echo "Installing air to $$AppBin"; \
		GOBIN="$$AppBin" go install github.com/air-verse/air@latest; \
	fi; \
	\
	$$AppBin/air

build:
	go mod vendor
	@mkdir -p $(BUILD_DIR)
	go build -mod=vendor -o $(BUILD_DIR)/main

.PHONY: all clean

all: $(foreach os,$(OS),$(foreach arch,$(ARCH),$(BUILD_DIR)/$(APP_NAME)-$(VERSION)-$(os)-$(arch)))

$(BUILD_DIR)/$(APP_NAME)-$(VERSION)-%:
	@mkdir -p $(BUILD_DIR)
	@echo "Building $@"
	OS=$$(echo $* | rev | cut -d'-' -f2 | rev)
	ARCH=$$(echo $* | rev | cut -d'-' -f1 | rev)
	GOOS=$$OS GOARCH=$$ARCH go build -o $@

clean:
	rm -rvf "vendor/bin"
	rm -rvf "$(BUILD_DIR)"
