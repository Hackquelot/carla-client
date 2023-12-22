CARLADIR=/home/roberto/GitHub/carla
BUILDDIR=$(CURDIR)/build
BINDIR=$(CURDIR)/bin
INSTALLDIR=$(CURDIR)/libcarla-install
TOOLCHAIN=$(CURDIR)/ToolChain.cmake

CC=/usr/bin/gcc-12
CXX=/usr/bin/g++-12
CXXFLAGS=-std=c++17 -pthread -fPIC -O3 -DNDEBUG -Werror -Wall -Wextra

SRCFILES = main.cpp

define log
	@echo "\033[1;35m$(1)\033[0m"
endef

default: $(BINDIR)/cpp_client

clean:
	@rm -rf $(BUILDDIR) $(BINDIR)

run: $(BINDIR)/cpp_client
	$(call log,Running C++ Client...)
	@$(BINDIR)/cpp_client $(ARGS)

run.only:
	$(call log,Running C++ Client...)
	@$(BINDIR)/cpp_client $(ARGS)

build: $(BINDIR)/cpp_client

$(BINDIR)/cpp_client: $(SRCFILES)
	$(call log,Compiling C++ Client...)
	@mkdir -p $(BINDIR)
	@$(CXX) $(CXXFLAGS) -I$(INSTALLDIR)/include -isystem $(INSTALLDIR)/include/system -L$(INSTALLDIR)/lib \
		-o $(BINDIR)/cpp_client $(SRCFILES) \
		-Wl,-Bstatic -lcarla_client -lrpc -lboost_filesystem -Wl,-Bdynamic \
		-lpng -ltiff -ljpeg -lRecast -lDetour -lDetourCrowd \
		`pkg-config --cflags --libs opencv4`

build_libcarla: $(TOOLCHAIN)
	@cd $(CARLADIR); make setup
	@mkdir -p $(BUILDDIR)
	$(call log,Compiling LibCarla.client...)
	@{ \
		cd $(BUILDDIR); \
		if [ ! -f "build.ninja" ]; then \
		cmake \
			-G "Ninja" \
			-DCMAKE_BUILD_TYPE=Client \
			-DLIBCARLA_BUILD_RELEASE=ON \
			-DLIBCARLA_BUILD_DEBUG=OFF \
			-DLIBCARLA_BUILD_TEST=OFF \
			-DCMAKE_TOOLCHAIN_FILE=$(TOOLCHAIN) \
			-DCMAKE_INSTALL_PREFIX=$(INSTALLDIR) \
			-DCMAKE_EXPORT_COMPILE_COMMANDS=1 \
			$(CARLADIR); \
	  fi; \
		ninja; \
		ninja install | grep -v "Up-to-date:"; \
	}

$(TOOLCHAIN):
	@echo "set(CMAKE_C_COMPILER $(CC))" > $(TOOLCHAIN)
	@echo "set(CMAKE_CXX_COMPILER $(CXX))" >> $(TOOLCHAIN)
	@echo "set(CMAKE_CXX_FLAGS \"\$${CMAKE_CXX_FLAGS} $(CXXFLAGS)\" CACHE STRING \"\" FORCE)" >> $(TOOLCHAIN)