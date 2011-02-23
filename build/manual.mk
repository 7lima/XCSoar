MANUAL_OUTPUT_DIR = $(OUT)/all/manual
GENERATED_DIR = $(OUT)/all/tex
TEX_FILES_EN = $(wildcard $(DOC)/manual/en/*.tex)
TEX_INCLUDES_EN = $(wildcard $(DOC)/manual/*.sty) $(wildcard $(DOC)/manual/en/*.sty)
FIGURES_EN = $(DOC)/manual/en/figures/*.png

SVG_ICON_LIST = \
	alt2_landable_airport \
	alt2_landable_field \
	alt_landable_airport \
	alt_landable_field \
	alt_reachable_airport \
	alt_reachable_field \
	winpilot_landable \
	winpilot_marginal \
	winpilot_reachable \
	map_turnpoint
SVG_ICONS = $(patsubst %,$(GENERATED_DIR)/icons/%.png,$(SVG_ICON_LIST))

TEX_INCLUDES_DE =  $(wildcard $(DOC)/manual/*.sty) $(wildcard $(DOC)/manual/de/*.sty)
FIGURES_DE = $(DOC)/manual/de/Bilder/*.png
 
TEX_FLAGS = -halt-on-error -interaction=nonstopmode

.PHONY: manual
manual: \
	$(MANUAL_OUTPUT_DIR)/XCSoar-manual.pdf \
	$(MANUAL_OUTPUT_DIR)/XCSoar-developer-manual.pdf \
	$(MANUAL_OUTPUT_DIR)/XCSoar-Handbuch.pdf

$(MANUAL_OUTPUT_DIR)/XCSoar-manual.pdf: $(DOC)/manual/en/XCSoar-manual.tex \
	$(TEX_FILES_EN) $(TEX_INCLUDES_EN) \
	$(FIGURES_EN) $(SVG_ICONS) | $(MANUAL_OUTPUT_DIR)/dirstamp
	# run TeX twice to make sure that all references are resolved
	cd $(<D) && pdflatex $(TEX_FLAGS) -output-directory $(abspath $(@D)) $(<F)
	cd $(<D) && pdflatex $(TEX_FLAGS) -output-directory $(abspath $(@D)) $(<F)

$(MANUAL_OUTPUT_DIR)/XCSoar-developer-manual.pdf: $(DOC)/manual/en/XCSoar-developer-manual.tex $(TEX_INCLUDES_EN) \
	$(FIGURES_EN) $(SVG_ICONS) | $(MANUAL_OUTPUT_DIR)/dirstamp
	# run TeX twice to make sure that all references are resolved
	cd $(<D) && pdflatex $(TEX_FLAGS) -output-directory $(abspath $(@D)) $(<F)
	cd $(<D) && pdflatex $(TEX_FLAGS) -output-directory $(abspath $(@D)) $(<F)

$(MANUAL_OUTPUT_DIR)/XCSoar-Handbuch.pdf: $(DOC)/manual/de/XCSoar-Handbuch.tex $(DOC)/manual/de/Blitzeinstieg.tex $(TEX_INCLUDES_DE) \
	$(FIGURES_DE) $(SVG_ICONS) | $(MANUAL_OUTPUT_DIR)/dirstamp
	# run TeX twice to make sure that all references are resolved
	cd $(<D) && pdflatex $(TEX_FLAGS) -output-directory $(abspath $(@D)) $(<F)
	cd $(<D) && pdflatex $(TEX_FLAGS) -output-directory $(abspath $(@D)) $(<F)

$(SVG_ICONS): $(GENERATED_DIR)/icons/%.png: $(topdir)/Data/icons/%.svg | $(GENERATED_DIR)/icons/dirstamp
	rsvg-convert -a -w 32 $< -o $@
