#LATEX = latex
LATEX = pdflatex
BIBTEX = bibtex
DVIPS = dvips
PS2PDF = ps2pdf
VIEWPDF = okular

BASE_NAME = wave_generation  # theory_of_operation

all: $(BASE_NAME).pdf

#byu-sow.pdf: byu-sow.tex
#	$(LATEX) $< && $(LATEX) $<

%.pdf: %.tex $(PNG_FILES) %.bbl
	$(LATEX) $< && $(LATEX) $< && $(LATEX) $<

$(BASE_NAME).bbl $(BASE_NAME).blg: $(BASE_NAME).bib $(BASE_NAME).aux
	$(BIBTEX) $(BASE_NAME)

$(BASE_NAME).aux: $(BASE_NAME).bib
	$(LATEX) $(BASE_NAME).tex

$(BASE_NAME).bib: $(BASE_NAME).tex
	$(LATEX) $(BASE_NAME).tex

$(BASE_NAME).dvi: $(BASE_NAME).tex
	$(LATEX) $< && $(LATEX) $<

$(BASE_NAME).ps: $(BASE_NAME).dvi
	$(DVIPS) -P pdf -f -t a4 $(BASE_NAME).dvi > $@

display: $(BASE_NAME).pdf
	$(VIEWPDF) $(BASE_NAME).pdf &

%.eps: %.dia
	cd `dirname $<` && dia `basename $<` -t eps

%.pdf: %.eps
	epstopdf $<

clean:
	rm -rf *.ps *.dvi *.log *.aux *.blg *.toc missfont.log *.bbl doxygen html latex *.out
