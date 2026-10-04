set(SOURCES
    arch.cc
    archdefs.h
    arch.h
    arch_pybindings.cc
    arch_pybindings.h
    cells.cc
    cells.h
    pack.cc
    viaduct_api.cc
    viaduct_api.h
    viaduct_constids.h
    viaduct_helpers.cc
    viaduct_helpers.h
    viaduct/example/constids.inc
    viaduct/example/example.cc
    viaduct/spartan3e/constids.inc
    viaduct/spartan3e/spartan3e.cc
    viaduct/spartan3e/spartan3e_ise.cc
    viaduct/spartan3e/spartan3e_ise.h
    viaduct/spartan3e/spartan3e_ise_carry.cc
    viaduct/spartan3e/spartan3e_ise_carry.h
    viaduct/spartan3e/spartan3e_ise_wide.cc
    viaduct/spartan3e/spartan3e_ise_wide.h
    viaduct/spartan3e/spartan3e_ise_clock.cc
    viaduct/spartan3e/spartan3e_ise_bram.cc
    viaduct/spartan3e/spartan3e_ise_clock_model.cc
    viaduct/spartan3e/spartan3e_ise_clock.h
    viaduct/spartan3e/spartan3e_ise_db.cc
    viaduct/spartan3e/spartan3e_ise_db.h
    viaduct/spartan3e/spartan3e_xdl.cc
    viaduct/spartan3e/spartan3e_xdl.h
    viaduct/fabulous/constids.inc
    viaduct/fabulous/fab_cfg.h
    viaduct/fabulous/fab_defs.h
    viaduct/fabulous/fabric_parsing.h
    viaduct/fabulous/fabulous.cc
    viaduct/fabulous/fasm.cc
    viaduct/fabulous/fasm.h
    viaduct/fabulous/bel_timing.cc
    viaduct/fabulous/bel_timing.h
    viaduct/fabulous/pack.cc
    viaduct/fabulous/pack.h
    viaduct/fabulous/validity_check.cc
    viaduct/fabulous/validity_check.h
    viaduct/fabulous/pcf.cc
    viaduct/fabulous/pcf.h
)

add_nextpnr_architecture(${family}
    CORE_SOURCES ${SOURCES}
    MAIN_SOURCE  main.cc
)

target_sources(nextpnr-${family}-chipdb PUBLIC chipdb.cc)
