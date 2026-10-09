set(SOURCES
    arch.cc arch.h archdefs.h
    arch_pybindings.cc arch_pybindings.h
    backend.cc backend.h implementation.h
    graph.cc site.cc pack.cc placement.cc physical.cc
    device.cc device.h
    ucf.cc ucf.h
    pack_helpers.cc pack_helpers.h constids.h constids.inc
    carry.cc carry.h wide.cc wide.h
    clock.cc clock.h
    bram.cc bram.h ff.h lutram.h mult.h srl.h
)
add_nextpnr_architecture(${family}
    CORE_SOURCES ${SOURCES}
    MAIN_SOURCE main.cc
)
