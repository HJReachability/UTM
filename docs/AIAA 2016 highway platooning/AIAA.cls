%%
%% Copyright notice.
%%
%% \CharacterTable
%%  {Upper-case    \A\B\C\D\E\F\G\H\I\J\K\L\M\N\O\P\Q\R\S\T\U\V\W\X\Y\Z
%%   Lower-case    \a\b\c\d\e\f\g\h\i\j\k\l\m\n\o\p\q\r\s\t\u\v\w\x\y\z
%%   Digits        \0\1\2\3\4\5\6\7\8\9
%%   Exclamation   \!     Double quote  \"     Hash (number) \#
%%   Dollar        \$     Percent       \%     Ampersand     \&
%%   Acute accent  \'     Left paren    \(     Right paren   \)
%%   Asterisk      \*     Plus          \+     Comma         \,
%%   Minus         \-     Point         \.     Solidus       \/
%%   Colon         \:     Semicolon     \;     Less than     \<
%%   Equals        \=     Greater than  \>     Question mark \?
%%   Commercial at \@     Left bracket  \[     Backslash     \\
%%   Right bracket \]     Circumflex    \^     Underscore    \_
%%   Grave accent  \`     Left brace    \{     Vertical bar  \|
%%   Right brace   \}     Tilde         \~}%
%%%  @LaTeX-file{
%%%     version         = "x",
%%%     date            = "0000/00/00",
%%%     author          = "",
%%%     revised by      = ""
%%%     copyright       = ""
%%%     address         = "",
%%%     FAX             = "",
%%%     email           = "",
%%%     abstract        = "LaTeX Template for AAS journal submissions",
%%%  }
\NeedsTeXFormat{LaTeX2e}[1996/12/01]%
\ProvidesClass{AIAA}%
\LoadClass[manuscript]{revtex4-1}[2001/08/03 v4.0 (http://publish.aps.org/revtex4/ for documentation)]
\def\class@name{revtex4-1}%
%\DeclareOption*{\PassOptionsToClass{\CurrentOption}{revtex4}}%
\ExecuteOptions{letterpaper,oneside,onecolumn,final}%
\ProcessOptions
\RequirePackage{latexsym}%
\RequirePackage{graphicx}%
\RequirePackage{verbatim}%
\usepackage[T1]{fontenc}

\def\normalsize{%
   \@setfontsize\normalsize\@xpt{12}%
   \abovedisplayskip 10\p@ \@plus2\p@ \@minus5\p@
   \belowdisplayskip \abovedisplayskip
   \abovedisplayshortskip  \abovedisplayskip
   \belowdisplayshortskip \abovedisplayskip
   \let\@listi\@listI
}%
\def\small{%
  \@setfontsize\small\@ixpt{10.5}%
  \abovedisplayskip 8.5\p@ \@plus3\p@ \@minus4\p@
  \belowdisplayskip \abovedisplayskip
  \abovedisplayshortskip \z@ \@plus2\p@
  \belowdisplayshortskip 4\p@ \@plus2\p@ \@minus2\p@
  \def\@listi{%
    \leftmargin\leftmargini
    \topsep 4\p@ \@plus2\p@ \@minus2\p@
    \parsep 2\p@ \@plus\p@ \@minus\p@
    \itemsep \parsep
  }%
}%
\def\footnotesize{%
  \@setfontsize\footnotesize\@viiipt{9.5pt}%
  \abovedisplayskip 6\p@ \@plus2\p@ \@minus4\p@
  \belowdisplayskip \abovedisplayskip
  \abovedisplayshortskip \z@ \@plus\p@
  \belowdisplayshortskip 3\p@ \@plus\p@ \@minus2\p@
  \def\@listi{%
    \leftmargin\leftmargini
    \topsep 3\p@ \@plus\p@ \@minus\p@
    \parsep 2\p@ \@plus\p@ \@minus\p@
    \itemsep \parsep
  }%
}%
\def\scriptsize{%
 \@setfontsize\scriptsize\@viipt\@viiipt
}%
\def\tiny{%
 \@setfontsize\tiny\@vpt\@vipt
}%
\def\large{%
 \@setfontsize\large\@xiipt{14pt}%
}%
\def\Large{%
 \@setfontsize\Large\@xivpt{18pt}%
}%
\def\LARGE{%
 \@setfontsize\LARGE\@xviipt{22pt}%
}%
\def\huge{%
 \@setfontsize\huge\@xxpt{25pt}%
}%
\def\Huge{%
 \@setfontsize\Huge\@xxvpt{30pt}%
}%
 \@ifxundefined\twoside@sw{\@booleanfalse\twoside@sw}{}%
 \twoside@sw{%
    \oddsidemargin  -20pt
    \evensidemargin -20pt
    \marginparwidth 107pt
 }{%
    \oddsidemargin  .15in
    \evensidemargin .15in
    \marginparwidth 30pt
 }%
\marginparsep 6pt
\topmargin -61pt
\headheight 25pt
\headsep 16pt
\topskip 10pt
\footskip 30pt
 \textheight = 56pc
\textwidth36pc
\columnsep 1.5pc
\columnseprule 0pt
\footnotesep 1pt
\skip\footins 39pt plus 4pt minus 2pt
\def\footnoterule{\kern-19pt\hrule width.5in\kern18.6pt\relax}%
\floatsep 12pt plus 2pt minus 2pt
\textfloatsep 20pt plus 2pt minus 4pt
\intextsep 12pt plus 2pt minus 2pt
\dblfloatsep 12pt plus 2pt minus 2pt
\dbltextfloatsep 20pt plus 2pt minus 4pt
\@fptop 0pt plus 1fil
\@fpsep 8pt plus 2fil
\@fpbot 0pt plus 1fil
\@dblfptop 0pt plus 1fil
\@dblfpsep 8pt plus 2fil
\@dblfpbot 0pt plus 1fil
\marginparpush 5pt
\parskip 0pt plus 1pt
\parindent 1.5pc
\emergencystretch8\p@
\partopsep 2pt plus 1pt minus 1pt
\leftmargini 25pt
\leftmarginii 22pt
\leftmarginiii 18.7pt
\leftmarginiv 17pt
\leftmarginv 10pt
\leftmarginvi 10pt
\def\@listI{%
  \leftmargin\leftmargini
  \parsep 4\p@ plus2\p@ minus\p@
  \topsep 8\p@ plus2\p@ minus4\p@
  \itemsep 4\p@ plus2\p@ minus\p@
}%
\labelsep 4pt
\def\@listii{%
  \leftmargin\leftmarginii
  \labelwidth\leftmarginii
  \advance\labelwidth-\labelsep
  \topsep 4\p@ plus2\p@ minus\p@
  \parsep 2\p@ plus\p@ minus\p@
  \itemsep \parsep
}%
\def\@listiii{%
  \leftmargin\leftmarginiii
  \labelwidth\leftmarginiii
  \advance\labelwidth-\labelsep
  \topsep 2\p@ plus\p@ minus\p@
  \parsep \z@
  \partopsep \p@ plus\z@ minus\p@
  \itemsep \topsep
}%
\def\@listiv{%
  \leftmargin\leftmarginiv
  \labelwidth\leftmarginiv
  \advance\labelwidth-\labelsep
}%
\def\@listv{%
  \leftmargin\leftmarginv
  \labelwidth\leftmarginv
  \advance\labelwidth-\labelsep
}%
\def\@listvi{%
  \leftmargin\leftmarginvi
  \labelwidth\leftmarginvi
  \advance\labelwidth-\labelsep
}%
%%%%%%%%%%%%%%%% from class file



\def\affils@present@group#1{%
 \begingroup
   \def\AF@temp{#1}%
   \@tempcnta\z@
   \let\AU@opr \@undefined
   \let\CO@opr \@undefined
   \def\AF@opr{\@affilID@count\AF@temp\@tempcnta}%
   \@AAC@list
   \@ifnum{\@tempcnta=\z@}{}{%
    \begingroup
     \frontmatter@above@affilgroup
     \set@listcomma@count\@tempcnta
     \let\AU@opr \@undefined
     \let\CO@opr \@undefined
     \def\AF@opr{\@affilID@match\AF@temp}%
     \@AAC@list
    \endgroup
    \begingroup
     \par
          \vskip-.5\baselineskip
     \frontmatter@above@affiliation
     \frontmatter@affiliationfont
     \let\\\frontmatter@addressnewline
     \@tempcnta\z@
     \@tfor\AF@temp:=#1\do{%
      \expandafter\@ifx\expandafter{\expandafter\z@\AF@temp}{}{%
       \advance\@tempcnta\@ne
      }%
     }%
     \@ifnum{\@tempcnta=\tw@}{%
      \let\@listcomma\relax
     }{}%
     \def@after@address
     \runinaddress@sw{%
     }{%
      \tightenlines@sw{
      }{%
       \parskip\z@
      }%
      \appdef\after@address\par
     }%
     \let\AFF@opr \@affil@group
     \do@affil@fromgroup\@AFF@list#1\relax
    \endgroup
   }%
   \par
 \endgroup
}%

\def\def@after@address{\def\after@address{\@listcomma\ \@listand}}%
\def\def@after@address@empty{\let\after@address\@empty}%


\def\frontmatter@thefootnote{%
 \altaffilletter@sw{\@alph}{\@fnsymbol}{\csname c@\@mpfn\endcsname}%
}%

\let\ltx@footnote@pop\@empty

\def\titleblock@produce{%
 \begingroup
  \ltx@footnote@pop
  \def\@mpfn{mpfootnote}%
  \def\thempfn{\thempfootnote}%
  \c@mpfootnote\z@
  \let\@makefnmark\frontmatter@makefnmark
  \frontmatter@setup
  \thispagestyle{titlepage}\label{FirstPage}%
  \frontmatter@title@produce
  \groupauthors@sw{%
   \frontmatter@author@produce@group
  }{%
   \frontmatter@author@produce@script
  }%
  \frontmatter@RRAPformat{%
   \expandafter\produce@RRAP\expandafter{\@date}%
   \expandafter\produce@RRAP\expandafter{\@received}%
   \expandafter\produce@RRAP\expandafter{\@revised}%
   \expandafter\produce@RRAP\expandafter{\@accepted}%
   \expandafter\produce@RRAP\expandafter{\@published}%
  }%
  \frontmatter@abstract@produce
  \@ifx@empty\@pacs{}{%
   \@pacs@produce\@pacs
  }%
  \@ifx@empty\@keywords{}{%
   \@keywords@produce\@keywords
  }%
  \par
  \frontmatter@finalspace
 \endgroup
}%


\def\frontmatter@title@produce{%
 \begingroup
  \frontmatter@title@above
  \frontmatter@title@format\LARGE
  \@title
  \unskip
  \phantomsection\expandafter\@argswap@val\expandafter{\@title}{\addcontentsline{toc}{title}}%
  \@ifx{\@title@aux\@title@aux@cleared}{}{%
   \expandafter\frontmatter@footnote\expandafter{\@title@aux}%
  }%
  \par
  \frontmatter@title@below
 \endgroup
}%



\renewenvironment{abstract}{%
  \aftermaketitle@chk{\begin{abstract}}%
  \global\setbox\absbox\vbox\bgroup
   \color@begingroup
   \columnwidth\textwidth
   \hsize\columnwidth
   \@parboxrestore
   \def\@mpfn{mpfootnote}\def\thempfn{\thempfootnote}\c@mpfootnote\z@
   \let\@footnotetext\frontmatter@footnotetext
   \minipagefootnote@init
   \let\@listdepth\@mplistdepth \@mplistdepth\z@
   \@minipagerestore
   \@setminipage
    %\frontmatter@abstractheading
    %\frontmatter@abstractfont%
    \vskip1pc
    \bfseries\hskip1pc
    \let\footnote\mini@note
}{%
    \par
    \unskip
    \minipagefootnote@here
    \@minipagefalse   %% added 24 May 89
    \color@endgroup
  \egroup
}%

\let\prep@absbox\@empty
\let\post@absbox\@empty

\def\frontmatter@abstract@produce{%
  \par
  \preprintsty@sw{%
   \do@output@MVL{%
    \vskip\frontmatter@preabstractspace
    \vskip200\p@%\@plus1fil
    \penalty-200\relax
    \vskip-200\p@%\@plus-1fil
   }%
  }{%
   \addvspace{\frontmatter@preabstractspace}%
  }%
   \begingroup
    \dimen@\baselineskip
    \setbox\z@\vtop{\unvcopy\absbox}%
    \advance\dimen@-\ht\z@\advance\dimen@-\prevdepth
    \@ifdim{\dimen@>\z@}{\vskip\dimen@}{}%
   \endgroup
   \begingroup
    \prep@absbox
    \unvbox\absbox
    \post@absbox
   \endgroup
  \@ifx{\@empty\mini@notes}{}{\mini@notes\par}%
  \addvspace\frontmatter@postabstractspace
}%

\def\maketitle{%
  \@author@finish
  \title@column\titleblock@produce
  \suppressfloats[t]%
  \let\and\relax
  \let\affiliation\@gobble
  \let\author\@gobble
  \let\@AAC@list\@empty
  \let\@AFF@list\@empty
  \let\@AFG@list\@empty
  \let\@AF@join\@AF@join@error
  \let\email\@gobble
  \let\@address\@empty
  \let\maketitle\relax
  \let\thanks\@gobble
  \titlepage@sw{%
  %\vfil
  %\clearpage
  \vspace*{12pt}
  }{}%
}%

\renewcommand\thetable{\@arabic\c@table}
\def\tablename{Table}
\def\figurename{Fig.}


\long\def\@makecaption#1#2{%
  \par
  \vskip\abovecaptionskip
  \begingroup
   \small\rmfamily
   \sbox\@tempboxa{%
    \let\\\heading@cr
     {\bf #1 #2}%
   }%
   \@ifdim{\wd\@tempboxa >\hsize}{%
    \begingroup
     \samepage
     \flushing
     \let\footnote\@footnotemark@gobble
     {\bf #1 #2}\par%
    \endgroup
   }{%
     \global \@minipagefalse
     \hb@xt@\hsize{\hfil\unhbox\@tempboxa\hfil}%
   }%
  \endgroup
  \vskip\belowcaptionskip
}%

\def\@make@capt@title#1#2{%
 \@ifx@empty\float@link{\@firstofone}{\expandafter\href\expandafter{\float@link}}%
  {#1}\@caption@fignum@sep#2%
}%

\def\@caption@fignum@sep{. }%
\def\setfloatlink{\def\float@link}%
\let\float@link\@empty

%%%%%%%%%%%% From APS4-1.rtx

\ifx\undefined\substyle@ext
 \def\@tempa{%
  \endinput
  \GenericWarning{I must be read in by REVTeX! (Bailing out)}%
 }%
 \expandafter\else
  \def\@tempa{}%
 \expandafter\fi\@tempa
 \class@info{RevTeX society APS selected}%
\DeclareOption{pra}{\change@journal{pra}}%
\DeclareOption{prb}{\change@journal{prb}}%
\DeclareOption{prc}{\change@journal{prc}}%
\DeclareOption{prd}{\change@journal{prd}}%
\DeclareOption{pre}{\change@journal{pre}}%
\DeclareOption{prl}{\change@journal{prl}}%
\DeclareOption{prstab}{\change@journal{prstab}}%
\DeclareOption{prstper}{\change@journal{prstper}}%
\DeclareOption{rmp}{\change@journal{rmp}}%
\def\ao{Appl.\  Opt.}%
\def\ap{Appl.\  Phys.}%
\def\apl{Appl.\ Phys.\ Lett.}%
\def\apj{Astrophys.\ J.}%
\def\bell{Bell Syst.\ Tech.\ J.}%
\def\jqe{IEEE J.\ Quantum Electron.}%
\def\assp{IEEE Trans.\ Acoust.\ Speech Signal Process.}%
\def\aprop{IEEE Trans.\ Antennas Propag.}%
\def\mtt{IEEE Trans.\ Microwave Theory Tech.}%
\def\iovs{Invest.\ Ophthalmol.\ Vis.\ Sci.}%
\def\jcp{J.\ Chem.\ Phys.}%
\def\jmo{J.\ Mod.\ Opt.}%
\def\josa{J.\ Opt.\ Soc.\ Am.}%
\def\josaa{J.\ Opt.\ Soc.\ Am.\ A}%
\def\josab{J.\ Opt.\ Soc.\ Am.\ B}%
\def\jpp{J.\ Phys.\ (Paris)}%
\def\nat{Nature (London)}%
\def\oc{Opt.\ Commun.}%
\def\ol{Opt.\ Lett.}%
\def\pl{Phys.\ Lett.}%
\def\pra{Phys.\ Rev.\ A}%
\def\prb{Phys.\ Rev.\ B}%
\def\prc{Phys.\ Rev.\ C}%
\def\prd{Phys.\ Rev.\ D}%
\def\pre{Phys.\ Rev.\ E}%
\def\prl{Phys.\ Rev.\ Lett.}%
\def\rmp{Rev.\ Mod.\ Phys.}%
\def\pspie{Proc.\ Soc.\ Photo-Opt.\ Instrum.\ Eng.}%
\def\sjqe{Sov.\ J.\ Quantum Electron.}%
\def\vr{Vision Res.}%
\def\@fnsymbol#1{%
 \ensuremath{%
  \ifcase#1\or
   *\or
   \dagger\or
   \ddagger\or
   \mathsection\or
   \mathparagraph\or
   **\or
   \dagger\dagger\or
   \ddagger\ddagger\else
   \mathsection\mathsection\or
   \mathparagraph\mathparagraph\or
   ***\or
   \dagger\dagger\dagger\or
   \ddagger\ddagger\ddagger\else
   \mathsection\mathsection\mathsection\or
   \mathparagraph\mathparagraph\mathparagraph\or
   \@ctrerr
  \fi
 }%
}%
\appdef\document@inithook{%
 \@ifxundefined\TextOrMath{%
  \DeclareRobustCommand\TextOrMath{\@ifmmode{\false@sw}{\true@sw}}%
 }{}%
}%
\let\thefootnote@latex\thefootnote
\clo@groupedaddress
\renewenvironment{titlepage}{%
  \let\wastwocol@sw\twocolumn@sw
  \onecolumngrid
  \newpage
  \thispagestyle{titlepage}%
  \c@page\z@
}{%
  \wastwocol@sw{\twocolumngrid}{\newpage}%
}%
\def\frontmatter@abstractheading{%
 \preprintsty@sw{%
  \begingroup
   \centering\large
   \abstractname
   \par
  \endgroup
 }{}%
}%
\def\frontmatter@abstractwidth{400\p@}%
\def\frontmatter@abstractfont{%
 \small
 \parindent1em\relax
 \adjust@abstractwidth
}%
\def\adjust@abstractwidth{%
 \dimen@\textwidth\advance\dimen@-\frontmatter@abstractwidth
 \divide\dimen@\tw@
 \galley@sw{%
  \advance\rightskip\tw@\dimen@
 }{%
  \advance\leftskip\dimen@
  \advance\rightskip\dimen@
 }%
 \@totalleftmargin\leftskip
}%
\@booleanfalse\preprintsty@sw
\@booleantrue\titlepage@sw
\appdef\setup@hook{%
  \preprintsty@sw{%
   \ps@preprint
   \def\frontmatter@abstractwidth{\textwidth}%
   \def\frontmatter@affiliationfont{\it}%
   \let\section\section@preprintsty
   \let\subsection\subsection@preprintsty
   \let\subsubsection\subsubsection@preprintsty
  }{%
   \ps@article
  }%
}%
\def\frontmatter@authorformat{%
 \skip@\@flushglue
 \@flushglue\z@ plus.3\hsize\relax
 \centering
 \advance\baselineskip\p@
 \parskip11.5\p@\relax
 \@flushglue\skip@
}%
\def\frontmatter@above@affilgroup{%
}%
\def\frontmatter@above@affiliation@script{%
 \skip@\@flushglue
 \@flushglue\z@ plus.3\hsize\relax
 \centering
 \@flushglue\skip@
 \addvspace{3.5\p@}%
}%
\def\frontmatter@above@affiliation{%
 \preprintsty@sw{}{%
 }%
}%
\def\frontmatter@affiliationfont{%
\it
}%
\def\frontmatter@collaboration@above{%
 \preprintsty@sw{%
 }{%
  \parskip1.5\p@\relax
 }%
}%
\def\frontmatter@setup{%
 \normalfont
}%
\def\frontmatter@title@above{\addvspace{6\p@}}%
\def\frontmatter@title@format{\large\bfseries\centering\parskip\z@skip}%
\def\frontmatter@title@below{}%
\def\@author@parskip{3\p@}%
\def\frontmatter@makefnmark{%
 \@textsuperscript{%
  \normalfont\@thefnmark
 }%
}%
\def\frontmatter@authorbelow{%
 \addvspace{3\p@}%
}%
\def\frontmatter@RRAP@format{%
  \small
  \centering
  \everypar{\hbox\bgroup(\@gobble@leavemode@uppercase}%
  \def\par{\@ifvmode{}{\unskip)\egroup\@@par}}%
}%
\def\punct@RRAP{;\egroup\ \hbox\bgroup}%
\def\@gobble@leavemode@uppercase#1#2{\expandafter\MakeTextUppercase}%
\def\frontmatter@PACS@format{%
   \addvspace{11\p@}%
   \footnotesize
   \adjust@abstractwidth
   \parindent\z@
   \parskip\z@skip
   \samepage
}%
\def\frontmatter@keys@format{%
   \footnotesize
   \adjust@abstractwidth
   \parindent\z@
   \samepage
}%
\def\ps@titlepage{%
  \def\@oddhead{%
   \hfill
   \preprint@sw{%
    \expandafter\produce@preprints\expandafter{\@preprint}%
   }{}%
  }%
  \let\@evenhead\@oddhead
  \def\@oddfoot{%
   \hb@xt@\z@{\byrevtex\hss}%
   \hfil
   \preprintsty@sw{\thepage}{}%
   \quad\checkindate
   \hfil
  }%
  \let\@evenfoot\@oddfoot
}%
\def\byrevtex{\byrevtex@sw{Typeset by REV\TeX}{}}%
\def\produce@preprints#1{%
 \vtop to \z@{%
  \def\baselinestretch{1}%
  \small
  \let\preprint\preprint@count
  \count@\z@
  #1%
  \@ifnum{\count@>\tw@}{%
   \hbox{%
    \let\preprint\preprint@hlist
    #1%
    \setbox\z@\lastbox
   }%
  }{%
   \let\preprint\preprint@cr
   \halign{\hfil##\cr#1\crcr}%
   \par
   \vss
  }%
 }%
}%
\def\preprint@cr#1{#1\cr}%
\def\preprint@count#1{\advance\count@\@ne}%
\def\preprint@hlist#1{#1\hbox{, }}%
\def\@seccntformat#1{\csname the#1\endcsname.\quad}%
\def\@hang@from#1#2#3{#1#2#3}%
\def\section{%
  \@startsection
    {section}%
    {1}%
    {\z@}%
    {0.4cm \@plus1ex \@minus .2ex}%
    {0.1cm}%
    {%
      \normalfont\small\bfseries
      \centering
    }%
}%
\def\@hangfrom@section#1#2#3{\@hangfrom{#1#2}{#3}}
\def\@hangfroms@section#1#2{#1{#2}}

\def\subsection{%
  \@startsection
    {subsection}%
    {2}%
    {\z@}%
    {0.4cm \@plus1ex \@minus .2ex}%
    {0.1cm}%
    {%
 \noindent\normalfont\small\bfseries
    }%
}%
\def\subsubsection{%
  \@startsection
    {subsubsection}%
    {3}%
    {\z@}%
    {.8cm \@plus1ex \@minus .2ex}%
    {.5cm}%
    {%
     \normalfont\small\itshape
    \centering
    }%
}%
\def\paragraph{%
  \@startsection
    {paragraph}%
    {4}%
    {\parindent}%
    {\z@}%
    {-1em}%
    {\normalfont\normalsize\itshape}%
}%
\def\subparagraph{%
  \@startsection
    {subparagraph}%
    {5}%
    {\parindent}%
    {3.25ex \@plus1ex \@minus .2ex}%
    {-1em}%
    {\normalfont\normalsize\bfseries}%
}%
\def\section@preprintsty{%
  \@startsection
    {section}%
    {1}%
    {\z@}%
    {0.8cm \@plus1ex \@minus .2ex}%
    {0.0001cm}%
    {%
      \normalfont\small\bfseries\centering
    }%
}%
\def\subsection@preprintsty{%
  \@startsection
    {subsection}%
    {2}%
    {-\parindent}%
    {.8cm \@plus1ex \@minus .2ex}%
    {.0001cm}%
    {%
     \normalfont\small\bfseries
    }%
}%
\def\subsubsection@preprintsty{%
  \@startsection
    {subsubsection}%
    {3}%
    {\z@}%
    {.8cm \@plus1ex \@minus .2ex}%
    {.0001cm}%
    {%
     \normalfont\small\itshape
    }%
}%
\let\frontmatter@footnote@produce\frontmatter@footnote@produce@footnote
\def\@pnumwidth{1.55em}%
\def\@tocrmarg {2.55em}%
\def\@dotsep{2}%
\def\ltxu@dotsep{4.5pt}%
\setcounter{tocdepth}{3}%
\def\tableofcontents{%
 \addtocontents{toc}{\string\tocdepth@munge}%
 \print@toc{toc}%
 \addtocontents{toc}{\string\tocdepth@restore}%
}%
\def\tocdepth@munge{%
  \let\l@section@saved\l@section
  \let\l@section\@gobble@tw@
}%
\def\@gobble@tw@#1#2{}%
\def\tocdepth@restore{%
  \let\l@section\l@section@saved
}%
\def\l@part#1#2{\addpenalty{\@secpenalty}%
 \begingroup
  \set@tocdim@pagenum\@tempboxa{#2}%
  \parindent \z@
  \rightskip\tocleft@pagenum plus 1fil\relax
  \skip@\parfillskip\parfillskip\z@
  \addvspace{2.25em plus\p@}%
  \large \bf %
  \leavevmode\ignorespaces#1\unskip\nobreak\hskip\skip@
  \hb@xt@\rightskip{\hfil\unhbox\@tempboxa}\hskip-\rightskip\hskip\z@skip
  \par
  \nobreak %
 \endgroup
}%
\def\tocleft@{\z@}%
\def\tocdim@min{5\p@}%
\def\l@section{%
 \l@@sections{}{section}% Implicit #3#4
}%
\def\l@f@section{%
 \addpenalty{\@secpenalty}%
 \addvspace{1.0em plus\p@}%
 %\bf
}%
\def\l@subsection{%
 \l@@sections{section}{subsection}% Implicit #3#4
}%
\def\l@subsubsection{%
 \l@@sections{subsection}{subsubsection}% Implicit #3#4
}%
\def\l@paragraph#1#2{}%
\def\l@subparagraph#1#2{}%
\let\toc@pre\toc@pre@auto
\let\toc@post\toc@post@auto
\@booleanfalse\raggedcolumn@sw
\def\tableft@skip@float{\z@ plus\hsize}%
\def\tabmid@skip@float{\@flushglue}%
\def\tabright@skip@float{\z@ plus\hsize}%
\def\array@row@pre@float{\hline\hline\noalign{\vskip\doublerulesep}}%
\def\array@row@pst@float{\noalign{\vskip\doublerulesep}\hline\hline}%
\long\def\@makefntext#1{%
 \def\baselinestretch{1}%
 \leftskip1em%
 \parindent1em%
 \noindent
 \nobreak\hskip-\leftskip
 \hb@xt@\leftskip{%
  \hss\@makefnmark\ %
 }%
 #1%
 \par
}%
\long\def\frontmatter@makefntext#1{%
 \def\baselinestretch{1}%
 \leftskip1em%
 \parindent1em%
 \noindent
 \nobreak\hskip-\leftskip
 \Hy@raisedlink{\hyper@anchorstart{frontmatter.\expandafter\the\csname c@\@mpfn\endcsname}\hyper@anchorend}%
 \hb@xt@\leftskip{%
  \hss\@makefnmark\ %
 }%
 #1%
 \par
}%
\prepdef\appendix{%
 \par
 \let\@hangfrom@section\@hangfrom@appendix
 \let\@sectioncntformat\@appendixcntformat
}%
\def\@hangfrom@appendix#1#2#3{%
 #1%
 \@if@empty{#2}{%
  #3%
 }{%
  #2\@if@empty{#3}{}{:\ #3}%
 }%
}%
\def\@hangfroms@appendix#1#2{%
 #1#2%
}%
\def\@appendixcntformat#1{\appendixname\ \csname the#1\endcsname}%
\@booleanfalse\authoryear@sw
\appdef\setup@hook{%
 \bibpunct{[}{]}{,}{n}{}{,}%
}%
 \def\pre@bibdata{\jobname\bibdata@app}%
\appdef\setup@hook{%
 \def\bibsection{%
  \par
  \onecolumngrid@push
  \begingroup
   \baselineskip26\p@
   \bib@device{\textwidth}{245.5\p@}%
  \endgroup
  \nobreak\@nobreaktrue
  \addvspace{19\p@}%
  \par
  \onecolumngrid@pop
 }%
}%
\def\bib@device#1#2{}%
% \hb@xt@\z@{%
%  \hb@xt@#1{%
%   \hfil
%   \phantomsection
%   \addcontentsline {toc}{section}{\protect\numberline{}\refname}%
%   \hb@xt@#2{%
%    \skip@\z@\@plus-1fil\relax
%                           \leaders\hrule height.25 \p@ depth.25 \p@ \hskip\z@\@plus1fil
%    \hskip\skip@
%    \hskip\z@\@plus0.125fil\leaders\hrule height.375\p@ depth.375\p@ \hskip\z@\@plus0.75fil \hskip\z@\@plus0.125fil
%    \hskip\skip@
%    \hskip\z@\@plus0.25 fil\leaders\hrule height.5  \p@ depth.5  \p@ \hskip\z@\@plus0.5 fil \hskip\z@\@plus0.25 fil
%    \hskip\skip@
%    \hskip\z@\@plus0.375fil\leaders\hrule height.625\p@ depth.625\p@ \hskip\z@\@plus0.25fil \hskip\z@\@plus0.375fil
% %  \hskip\skip@
% %  \hfil
%   }%
%   \hfil
%  }%
%  \hss
% }%
%}%
\appdef\setup@hook{%
 \let\bibpreamble\@empty
 \bibsep\z@\relax
 \def\newblock{\ }%
}%
\appdef\setup@hook{%
 \def\bibfont{%
  \small
  \@clubpenalty\clubpenalty
 }%
}%
\renewenvironment{theindex}{%
 \columnseprule \z@
 \columnsep 35\p@
 \c@secnumdepth-\maxdimen
 \onecolumngrid@push
 \section{\indexname}%
 \thispagestyle{plain}%
 \parindent\z@
 \parskip\z@ plus.3\p@\relax
 \let\item\@idxitem
 \onecolumngrid@pop
}{%
}%
\def\@idxitem{\par\hangindent 40\p@}%
\def\subitem{\par\hangindent 40\p@ \hspace*{20\p@}}%
\def\subsubitem{\par\hangindent 40\p@ \hspace*{30\p@}}%
\def\indexspace{\par \vskip 10\p@ plus5\p@ minus3\p@\relax}%
\def\@journal@default{pra}%
\def\@pointsize@default{10}%
\def\rtx@apspra{%
 \class@info{APS journal PRA selected}%
}%
\def\rtx@apsprb{%
 \class@info{APS journal PRB selected}%
 \let\frontmatter@footnote@produce\frontmatter@footnote@produce@endnote
 \appdef\setup@hook{%
  \bibpunct{}{}{,}{s}{}{\textsuperscript{,}}%
  \let\onlinecite\rev@citealpnum
 }%
 \def\pre@bibdata{\jobname\bibdata@app}%
 \let\place@bibnumber\place@bibnumber@sup
}%
\def\rtx@apsprc{%
 \class@info{APS journal PRC selected}%
}%
\def\rtx@apsprd{%
 \class@info{APS journal PRD selected}%
}%
\def\rtx@apspre{%
 \class@info{APS journal PRE selected}%
}%
\def\rtx@apsprl{%
 \class@info{APS journal PRL selected}%
 \let\frontmatter@footnote@produce\frontmatter@footnote@produce@endnote
 \@booleanfalse\acknowledgments@sw
 \appdef\setup@hook{%
  \def\bibsection{%
   \par
   \begingroup
    \baselineskip26\p@
    \bib@device{\hsize}{72\p@}%
   \endgroup
   \nobreak\@nobreaktrue
   \addvspace{19\p@}%
  }%
 }%
\appdef\setup@hook{%
  \lengthcheck@sw{%
   \RequirePackage{times}%
  }{}%
}%
 \c@secnumdepth=-\maxdimen
 \appdef\setup@hook{%
  \@ifnum{\@pointsize=10\relax}{%
   \lengthcheck@sw{%
    \def\large{%
     \@setfontsize\large{12.5}{14\p@}%
    }%
    \def\normalsize{%
     \@setfontsize\normalsize{10.5}\@xiipt
     \abovedisplayskip 6\p@ \@plus6\p@ \@minus5\p@
     \belowdisplayskip \abovedisplayskip
     \abovedisplayshortskip  \abovedisplayskip
     \belowdisplayshortskip \abovedisplayskip
     \let\@listi\@listI
    }%
    \def\small{%
     \@setfontsize\small{9.5}\@xipt
     \abovedisplayskip 5\p@ \@plus5\p@ \@minus4\p@
     \belowdisplayskip \abovedisplayskip
     \abovedisplayshortskip  \abovedisplayskip
     \belowdisplayshortskip \abovedisplayskip
     \let\@listi\@listI
    }%
    \DeclareMathSizes{12.5}{12.5}{9}{6}%
    \DeclareMathSizes{10.5}{10.5}{7.5}{5}%
    \DeclareMathSizes{9.5}{9.5}{7.0}{5}%
   }{%
    \def\normalsize{%
     \@setfontsize\normalsize\@xpt\@xiipt
     \abovedisplayskip 10\p@ \@plus2\p@ \@minus5\p@
     \belowdisplayskip \abovedisplayskip
     \abovedisplayshortskip  \abovedisplayskip
     \belowdisplayshortskip \abovedisplayskip
     \let\@listi\@listI
    }%
   }%
  }{}%
 }%
 \textheight = 694.0\p@
}%
\def\rtx@apsprstper{%
 \class@info{APS journal PRST-PER selected}%
 \rtx@apsprb
 \@booleantrue\longbibliography@sw
}%
\def\rtx@apsprstab{%
 \class@info{APS journal PRSTAB selected}%
}%
\@booleantrue\footinbib@sw
\appdef\@bibdataout@rev{\@bibdataout@aps}%
\def\@bibdataout@aps{%
 \immediate\write\@bibdataout{%
  @CONTROL{%
   apsrev41Control%
   \longbibliography@sw{%
    ,author="00",editor="1",pages="1",title="0",year="0"%
   }{%
    ,author="08",editor="1",pages="0",title="",year="1"%
   }%
  }%
 }%
 \if@filesw
  \immediate\write\@auxout{\string\citation{apsrev41Control}}%
 \fi
}%
\let\place@bibnumber\place@bibnumber@inl
\def\@bibstyle{apsrev\substyle@post}%
\appdef\setup@hook{%
 \@ifx{\place@bibnumber\place@bibnumber@sup}{%
  \footinbib@sw{}{%
   \class@warn{Citations are superscript numbers: footnotes must be endnotes; changing to that configuration}%
   \@booleantrue\footinbib@sw
  }%
 }{}%
}%

\def\substyle@post{4-1}%

%%%\def\NAT@BibitemShut#1{%
%%%\def\@bibstop{#1}%
%%% \let\bibitem@Stop\bibitemStop
%%% \let\bibitem@NoStop\bibitemNoStop
%%% \@ifx{\bibitemShut\relax}{\let\@bibitemShut\@empty}{%
%%% \expandafter\def\expandafter\@bibitemShut\expandafter{\bibitemShut}%
%%% }%
%%% }%

\let\@bibitemShut\relax



\def\baselinestretch{2}% For double spacing


\endinput
%%
%% End of file `AIAA.cls'.
