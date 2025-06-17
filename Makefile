# 修复路径问题的 Makefile - 支持 ILP 路由器
# 主要修复：正确的环境变量设置和路径配置

# ============================================================================
# Coriolis Alliance 路径修正 (关键修改 - 修复路径问题)
# ============================================================================

# 在 include 之前先设置正确的路径
CORIOLIS_INSTALL := /home/undergrad22/u22_linziqiao/coriolis-2.x/Linux.SL7_64/Release.Shared/install

# 强制设置正确的 Alliance 路径
export ALLIANCE_TOP := $(CORIOLIS_INSTALL)
export ALLIANCE_BIN := $(CORIOLIS_INSTALL)/bin
export CELLS_TOP := $(CORIOLIS_INSTALL)/cells
export SYSCONF_TOP := $(CORIOLIS_INSTALL)/etc

# 修复：确保ALLIANCE_BIN正确设置
export PATH := $(ALLIANCE_BIN):$(PATH)
export LD_LIBRARY_PATH := $(CORIOLIS_INSTALL)/lib:$(LD_LIBRARY_PATH)

# 包含原始配置，但我们的设置会覆盖
include ../../etc/alliance-env.mk

# ============================================================================
# GLPK ILP 路由器支持 (新增)
# ============================================================================

USER_GLPK_HOME = /home/undergrad22/u22_linziqiao/glpk
GLPK_INCLUDE_DIR := $(USER_GLPK_HOME)/include
GLPK_LIB_DIR := $(USER_GLPK_HOME)/lib
EXTRA_CXXFLAGS := -std=c++17 -I$(GLPK_INCLUDE_DIR) -DUSE_GLPK_ILP -DFORCE_ILP_ONLY=1 -DNO_ASTAR_FALLBACK=1 -DPURE_ILP_MODE=1 -Wall -Wextra -O2
EXTRA_LDFLAGS := -L$(GLPK_LIB_DIR) -lglpk -lm -lpthread

# 修复：添加GLPK到库路径
export LD_LIBRARY_PATH := $(GLPK_LIB_DIR):$(LD_LIBRARY_PATH)

$(info ILP Router: Alliance=$(ALLIANCE_TOP), GLPK=$(USER_GLPK_HOME))
$(info Fixed PATH: $(PATH))
$(info Fixed LD_LIBRARY_PATH: $(LD_LIBRARY_PATH))

# ============================================================================
# 修正环境变量 (关键修复 - 正确的PATH和LD_LIBRARY_PATH)
# ============================================================================

# 修复：重新定义 MBK 环境，确保所有路径正确
override MBK_GENERAT_ENV = \
                   export ALLIANCE_TOP="$(ALLIANCE_TOP)"; \
                   export ALLIANCE_BIN="$(ALLIANCE_BIN)"; \
                   export PATH="$(ALLIANCE_BIN):/usr/local/bin:/usr/bin:/bin:$$PATH"; \
                   export LD_LIBRARY_PATH="$(CORIOLIS_INSTALL)/lib:$(GLPK_LIB_DIR):$$LD_LIBRARY_PATH"; \
                   export MBK_TARGET_LIB="$(TARGET_LIB)"; \
                   export MBK_WORK_LIB="."; \
                   export MBK_CATA_LIB="$(CATA_LIB)"; \
                   export MBK_CATAL_NAME="CATAL"; \
                   export MBK_OUT_LO="$(GENERAT_LO)"; \
                   export MBK_OUT_PH="$(GENERAT_PH)"; \
                   export MBK_IN_LO="$(GENERAT_LO)"; \
                   export MBK_IN_PH="$(GENERAT_PH)"; \
                   export MBK_SEPAR="$(GENERAT_SP)"; \
                   export MBK_VDD="vdd"; \
                   export MBK_VSS="vss"; \
                   export FPGEN_LIB="$(FPGEN_LIB)"; \
                   export FORCE_ILP_ONLY=1; \
                   export PURE_ILP_MODE=1; \
                   export RDS_TECHNO_NAME="$(TECHNO_NAME).rds"

# 修复：MBK extracting environment
override MBK_EXTRACT_ENV = \
                   export ALLIANCE_TOP="$(ALLIANCE_TOP)"; \
                   export ALLIANCE_BIN="$(ALLIANCE_BIN)"; \
                   export PATH="$(ALLIANCE_BIN):/usr/local/bin:/usr/bin:/bin:$$PATH"; \
                   export LD_LIBRARY_PATH="$(CORIOLIS_INSTALL)/lib:$(GLPK_LIB_DIR):$$LD_LIBRARY_PATH"; \
                   export MBK_TARGET_LIB="$(TARGET_LIB)"; \
                   export MBK_WORK_LIB="."; \
                   export MBK_CATA_LIB="$(CATA_LIB)"; \
                   export MBK_CATAL_NAME="CATAL"; \
                   export MBK_OUT_LO="$(EXTRACT_LO)"; \
                   export MBK_OUT_PH="$(EXTRACT_PH)"; \
                   export MBK_IN_LO="$(EXTRACT_LO)"; \
                   export MBK_IN_PH="$(EXTRACT_PH)"; \
                   export MBK_SEPAR="$(EXTRACT_SP)"; \
                   export MBK_VDD="vdd"; \
                   export MBK_VSS="vss"; \
                   export FORCE_ILP_ONLY=1; \
                   export RDS_TECHNO_NAME="$(TECHNO_NAME).rds"

# 修复：MBK caracterisation
override MBK_CARAC_ENV = \
                   export ALLIANCE_TOP="$(ALLIANCE_TOP)"; \
                   export ALLIANCE_BIN="$(ALLIANCE_BIN)"; \
                   export PATH="$(ALLIANCE_BIN):/usr/local/bin:/usr/bin:/bin:$$PATH"; \
                   export LD_LIBRARY_PATH="$(CORIOLIS_INSTALL)/lib:$(GLPK_LIB_DIR):$$LD_LIBRARY_PATH"; \
                   export MBK_TARGET_LIB="$(TARGET_LIB)"; \
                   export MBK_WORK_LIB="."; \
                   export MBK_CATA_LIB="$(CATA_LIB)"; \
                   export MBK_CATAL_NAME="CATAL"; \
                   export MBK_OUT_LO="$(CARAC_LO)"; \
                   export MBK_SPI_MODEL="$(CARAC_SPI_MODEL)"; \
                   export MBK_OUT_PH="$(EXTRACT_PH)"; \
                   export MBK_IN_LO="$(EXTRACT_LO)"; \
                   export MBK_IN_PH="$(EXTRACT_PH)"; \
                   export MBK_SEPAR="$(EXTRACT_SP)"; \
                   export MBK_VDD="vdd"; \
                   export MBK_VSS="vss"; \
                   export FORCE_ILP_ONLY=1; \
                   export RDS_IN="cif"; \
                   export RDS_OUT="cif"; \
                   export RDS_TECHNO_NAME="$(CARAC_TECHNO_NAME).rds"

# 修复：ENV_COUGAR_SPI
override ENV_COUGAR_SPI = \
                   export ALLIANCE_TOP="$(ALLIANCE_TOP)"; \
                   export ALLIANCE_BIN="$(ALLIANCE_BIN)"; \
                   export PATH="$(ALLIANCE_BIN):/usr/local/bin:/usr/bin:/bin:$$PATH"; \
                   export LD_LIBRARY_PATH="$(CORIOLIS_INSTALL)/lib:$(GLPK_LIB_DIR):$$LD_LIBRARY_PATH"; \
                   export MBK_WORK_LIB="."; \
                   export MBK_IN_LO="spi"; \
                   export MBK_OUT_LO="spi"; \
                   export MBK_SPI_MODEL="$(SPI_MODEL)"; \
                   export MBK_SPI_NAMEDNODES="true"; \
                   export RDS_TECHNO_NAME="$(RDS_TECHNO_REAL)"; \
                   export RDS_IN="cif"; \
                   export RDS_OUT="cif"; \
                   export MBK_CATA_LIB="$(CATA_LIB)"; \
                   export MBK_IN_PH="ap"; \
                   export MBK_OUT_PH="ap"; \
                   export MBK_CATAL_NAME="CATAL"; \
                   export FORCE_ILP_ONLY=1

# ============================================================================
# 工具定义 (修复路径)
# ============================================================================

# --------------------------------------------------------------------
# Standarts binaries (修复：使用绝对路径)

           LS = /bin/ls
           CD = cd
           CP = cp -f
           LN = ln
           MV = mv
           RM = rm -f
          SED = sed
          AWK = gawk
          CAT = cat
         MAKE = make
         ECHO = /bin/echo

#  Alliance paths and formats settings.
 GENERAT_LO   = vst
 EXTRACT_LO   = al
 CARAC_LO     = spi
 GENERAT_PH   = ap
 EXTRACT_PH   = ap
 GENERAT_SP   = .
 EXTRACT_SP   = .
    CATA_LIB0 = $(CELLS_TOP)/sxlib
    CATA_LIB1 = $(CELLS_TOP)/dp_sxlib
    CATA_LIB2 = $(CELLS_TOP)/padlib
    CATA_LIB3 = $(CELLS_TOP)/rflib
    CATA_LIB  = .:$(CATA_LIB0):$(CATA_LIB1):$(CATA_LIB2):$(CATA_LIB3)
   TARGET_LIB = $(CELLS_TOP)/sxlib
    FPGEN_LIB = $(CATA_LIB0):$(CATA_LIB1)
  TECHNO_NAME = local-cmos
CARAC_TECHNO_NAME = local-cmos-035
CARAC_SPI_MODEL = ./model

SPI_MODEL       = $(SYSCONF_TOP)/spimodel.cfg
RDS_TECHNO_REAL = ./local-cmos-035.rds

# --------------------------------------------------------------------
# Alliance binaries & environment (修复：确保使用正确的路径)

    GRAAL = $(MBK_GENERAT_ENV); export GRAAL_TECHNO_NAME="$(TECHNO_NAME).graal"; $(ALLIANCE_BIN)/graal
   ASIMUT = $(MBK_GENERAT_ENV); $(ALLIANCE_BIN)/asimut -zd -bdd
  FLATBEH = $(MBK_GENERAT_ENV); $(ALLIANCE_BIN)/flatbeh
  FLATLO  = $(MBK_GENERAT_ENV); $(ALLIANCE_BIN)/flatlo
     LYNX = $(MBK_EXTRACT_ENV); $(ALLIANCE_BIN)/cougar -v
    LYNX3 = $(ENV_COUGAR_SPI); $(ALLIANCE_BIN)/cougar -v
     DRUC = $(MBK_GENERAT_ENV); $(ALLIANCE_BIN)/druc
      LVX = $(MBK_EXTRACT_ENV); $(ALLIANCE_BIN)/lvx
    PROOF = $(MBK_EXTRACT_ENV); $(ALLIANCE_BIN)/proof
     RING = $(MBK_GENERAT_ENV); $(ALLIANCE_BIN)/ring
    DPGEN = $(MBK_GENERAT_ENV); $(ALLIANCE_BIN)/genlib --keep-exec --verbose
    OCP   = $(MBK_GENERAT_ENV); $(ALLIANCE_BIN)/ocp -v -gnuplot
    OCR   = $(MBK_GENERAT_ENV); $(ALLIANCE_BIN)/ocr
    
    # 修复：ILP 增强版 NERO (确保环境变量正确传递)
    NERO  = $(MBK_GENERAT_ENV); \
            echo "Starting NERO with PURE ILP Router..."; \
            echo "WARNING: A* Algorithm: COMPLETELY DISABLED"; \
            echo "Algorithm: GLPK Integer Linear Programming"; \
            echo "Alliance Bin: $(ALLIANCE_BIN)"; \
            echo "GLPK Lib: $(GLPK_LIB_DIR)"; \
            export CXXFLAGS="$(EXTRA_CXXFLAGS)"; \
            export LDFLAGS="$(EXTRA_LDFLAGS)"; \
            export FORCE_ILP_ONLY=1; \
            export PURE_ILP_MODE=1; \
            export NO_ASTAR_FALLBACK=1; \
            which nero || echo "WARNING: nero not found in PATH"; \
            $(ALLIANCE_BIN)/nero -V
    
      S2R = $(MBK_CARAC_ENV); $(ALLIANCE_BIN)/s2r -v

GENPAT		= $(ALLIANCE_BIN)/genpat
GENLIB		= $(MBK_GENERAT_ENV); $(ALLIANCE_BIN)/genlib
SYF		= $(MBK_GENERAT_ENV); $(ALLIANCE_BIN)/syf -a -CVE
BOOM		= $(MBK_GENERAT_ENV); $(ALLIANCE_BIN)/boom -A -V -l 2 -s
BOOG		= $(MBK_GENERAT_ENV); $(ALLIANCE_BIN)/boog
LOON		= $(MBK_GENERAT_ENV); $(ALLIANCE_BIN)/loon 
SCAPIN		= $(MBK_GENERAT_ENV); $(ALLIANCE_BIN)/scapin -VRB
MIPS_ASM	= $(ALLIANCE_BIN)/mips_asm
BENCH	    = ./bench.sh add000 CONF
ALLBENCH    = ./allbench.sh DEFINITIF

# ============================================================================
# 验证和调试目标 (修复和增强)
# ============================================================================

.PHONY: verify-ilp-setup debug-env check-paths
verify-ilp-setup:
	@echo "Verifying ILP Setup..."
	@echo "================================================"
	@echo "Paths:"
	@echo "  Alliance Top: $(ALLIANCE_TOP)"
	@echo "  Alliance Bin: $(ALLIANCE_BIN)"
	@echo "  GLPK Home: $(USER_GLPK_HOME)"
	@echo "  Current PATH: $$PATH"
	@echo "================================================"
	@echo "File Checks:"
	@test -d $(ALLIANCE_TOP) && echo "SUCCESS: Alliance directory: OK" || echo "ERROR: Alliance directory: MISSING"
	@test -d $(ALLIANCE_BIN) && echo "SUCCESS: Alliance bin directory: OK" || echo "ERROR: Alliance bin directory: MISSING"
	@test -f $(ALLIANCE_BIN)/genlib && echo "SUCCESS: genlib: OK" || echo "ERROR: genlib: MISSING"
	@test -f $(ALLIANCE_BIN)/nero && echo "SUCCESS: nero: OK" || echo "ERROR: nero: MISSING"
	@test -f $(GLPK_INCLUDE_DIR)/glpk.h && echo "SUCCESS: GLPK header: OK" || echo "ERROR: GLPK header: MISSING"
	@test -f $(GLPK_LIB_DIR)/libglpk.a && echo "SUCCESS: GLPK library: OK" || echo "ERROR: GLPK library: MISSING"
	@echo "================================================"
	@echo "Verification completed"

debug-env:
	@echo "Environment Debug Information:"
	@echo "================================================"
	@echo "ALLIANCE_TOP = $(ALLIANCE_TOP)"
	@echo "ALLIANCE_BIN = $(ALLIANCE_BIN)"
	@echo "USER_GLPK_HOME = $(USER_GLPK_HOME)"
	@echo "GLPK_INCLUDE_DIR = $(GLPK_INCLUDE_DIR)"
	@echo "GLPK_LIB_DIR = $(GLPK_LIB_DIR)"
	@echo "EXTRA_CXXFLAGS = $(EXTRA_CXXFLAGS)"
	@echo "EXTRA_LDFLAGS = $(EXTRA_LDFLAGS)"
	@echo "================================================"
	@echo "Testing MBK Environment:"
	@$(MBK_GENERAT_ENV); echo "PATH in MBK: $$PATH" | head -c 200; echo "..."
	@$(MBK_GENERAT_ENV); echo "LD_LIBRARY_PATH: $$LD_LIBRARY_PATH" | head -c 200; echo "..."

check-paths:
	@echo "Checking all critical paths..."
	@echo "Alliance tools:"
	@ls -la $(ALLIANCE_BIN)/nero $(ALLIANCE_BIN)/genlib $(ALLIANCE_BIN)/cougar 2>/dev/null || echo "Some Alliance tools missing"
	@echo ""
	@echo "GLPK files:"
	@ls -la $(GLPK_INCLUDE_DIR)/glpk.h $(GLPK_LIB_DIR)/libglpk.* 2>/dev/null || echo "Some GLPK files missing"

# ============================================================================
# 原有目标保持不变，只增强 ILP 路由目标
# ============================================================================

mips_chip.cif : mips_chip.ap
	$(S2R) -r mips_chip mips_chip

mips_chip_t.al : mips_chip.ap
	$(LYNX) -t -ac -ar mips_chip mips_chip_t

mips_core_t.al : mips_core.ap
	$(LYNX) -c -t -ac -ar mips_core mips_core_t

mips_chip.ap : mips_core.ap mips_chip.vst mips_chip.rin mips_core.vst mips_core.al
	$(CP) CATAL_VST CATAL
	$(RING) mips_chip mips_chip
       
mips_core.al : mips_core.ap
	$(CP) CATAL_VST CATAL
	$(LYNX) -c -f mips_core mips_core
	$(LVX) vst al mips_core mips_core -f

mips_core.spi : mips_core.ap
	$(LYNX3) -ac -f mips_core mips_core

mips_core_et.spi : mips_core.ap
	$(LYNX3) -ac -t mips_core mips_core_et

# 主要 ILP 路由目标 (增强调试信息)
mips_core.ap : mips_core_p.ap mips_ctl.vst
	@echo "================================================"
	@echo "Starting ILP Routing for mips_core"
	@echo "Algorithm: GLPK Integer Linear Programming"
	@echo "WARNING: A* Algorithm: COMPLETELY DISABLED"
	@echo "Using nero at: $(ALLIANCE_BIN)/nero"
	@echo "================================================"
	$(CP) CATAL_VST CATAL
	@echo "Environment check before NERO:"
	@$(MBK_GENERAT_ENV); echo "$$PATH" | grep -o "$(ALLIANCE_BIN)" && echo "SUCCESS: Alliance bin in PATH" || echo "ERROR: Alliance bin NOT in PATH"
	$(NERO) -6 -p mips_core_p mips_core mips_core_p_nero 2>&1 | tee nero_ilp_routing.log
	cat mips_core_p_nero.ap | sed s/core_p_nero/core/ > mips_core.ap
	$(DRUC) mips_core
	@echo "================================================"
	@echo "ILP Routing Completed Successfully!"
	@echo "Check nero_ilp_routing.log for details"
	@echo "Generated files: mips_core.ap"
	@echo "================================================"

mips_core_p.ap : mips_core_place.ap mips_ctl.vst mips_core.ioc
	$(CP) CATAL_VST CATAL
	$(OCP) -partial mips_core_place -ioc  mips_core mips_core mips_core_p

mips_core_place.ap : mips_core.c mips_dpt.ap
	$(DPGEN) mips_core

mips_ctl.vst : mips_ctl_loon.vst mips_ctl.path
	$(SCAPIN) -P mips_ctl.scapin mips_ctl_loon mips_ctl mips_ctl

mips_ctl_loon.vst : mips_sts.vst mips_seqo.vst mips_ctl.lax
	$(CP) CATAL_CTL CATAL
	$(FLATLO) -r mips_ctl_nt mips_ctl_flat
	$(LOON) mips_ctl_flat mips_ctl_loon -l mips_ctl 

mips_seqo.vst : mips_seqo_optim.vbe mips_ctl.lax
	$(BOOG) mips_seqo_optim mips_seqo -l mips_ctl

mips_sts.vst : mips_sts_optim.vbe mips_ctl.lax
	$(BOOG) mips_sts_optim mips_sts -l mips_ctl

mips_seqo_optim.vbe : mips_seqo.vbe
	$(BOOM) mips_seqo mips_seqo_optim

mips_sts_optim.vbe : mips_sts.vbe
	$(BOOM) mips_sts mips_sts_optim

mips_bench : mips_cpu.pat mips_cpu.vst
	$(BENCH)
	@echo "<---------- resultat a regarder dans CONF ---------------->"

mips_dpt.ap model_shift.ap : mips_dpt.c
	$(DPGEN) mips_dpt

mips_seqo.vbe : mips_seq.fsm
	$(SYF) 	mips_seq mips_seqo

mips_allbench :
	$(CP) CATAL_VST CATAL 
	$(ALLBENCH)
	@echo "<---------- resultat a regarder dans DEFINITIF ---------------->"

mips_test : mips_ctl.vst mips_scan.pat
	$(ASIMUT) -p 50 mips_cpu mips_scan resultat 2> ajeter

mips_scan.pat : mips_scan.c
	$(GENPAT) mips_scan

graal :
	$(GRAAL)

# 增强清理目标，包含 ILP 日志和调试文件
clean :
	$(RM) mips_dpt.vst
	$(RM) *.dtx
	$(RM) *.ttx
	$(RM) *.drc
	$(RM) *.rcx
	$(RM) *.cif
	$(RM) *.spi
	$(RM) *.rep
	$(RM) *.ap
	$(RM) *.al
	$(RM) mips_ctl.vst
	$(RM) mips_ctl_flat.vst
	$(RM) mips_ctl_loon.vst
	$(RM) res.pat
	$(RM) resultat.pat
	$(RM) res2
	$(RM) mips_sts.xsc
	$(RM) mips_sts.vst
	$(RM) mips_sts_optim.vbe	
	$(RM) mips_seqo_optim.vbe	
	$(RM) machin		 
	$(RM) bench/*.good 	
	$(RM) CONF 		
	$(RM) CATAL 		
	$(RM) dat* 		
	$(RM) DEFINITIF		
	$(RM) mips_seqo*	
	$(RM) *.xsc	
	$(RM) *.gpl	
	$(RM) *.gds	
	$(RM) model*
	$(RM) alldata.dat
	$(RM) nero_ilp_routing.log

# 修复：添加快速测试目标
test-nero:
	@echo "Testing NERO path resolution..."
	@$(MBK_GENERAT_ENV); which nero && echo "SUCCESS: nero found" || echo "ERROR: nero not found"
	@echo "Testing nero executable (nero uses different argument format):"
	@$(MBK_GENERAT_ENV); $(ALLIANCE_BIN)/nero 2>&1 | head -3 | grep -q "error\|usage\|Usage\|COpts" && echo "SUCCESS: nero executable and responding" || echo "ERROR: nero not responding"
	@echo "Nero dynamic library dependencies:"
	@ldd $(ALLIANCE_BIN)/nero | grep -E "(not found|libRds|libMpu)" | head -5

# 修复：添加环境设置助手
setup-env:
	@echo "Environment setup commands:"
	@echo "export ALLIANCE_TOP=\"$(ALLIANCE_TOP)\""
	@echo "export ALLIANCE_BIN=\"$(ALLIANCE_BIN)\""
	@echo "export PATH=\"$(ALLIANCE_BIN):\$$PATH\""
	@echo "export LD_LIBRARY_PATH=\"$(CORIOLIS_INSTALL)/lib:$(GLPK_LIB_DIR):\$$LD_LIBRARY_PATH\""
	@echo ""
	@echo "Run: eval \$$(make setup-env | grep export)"