import os
# import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import seaborn as sns
from cycler import cycler
import scienceplots  # noqa: F401

cm = 1/2.54  # centimeters in inches
width_two_cols = 18.2*cm
width_one_col = 8.89*cm

# line cyclers adapted to colourblind people
# ["#E69F00", "#56B4E9", "#009E73", "#0072B2", "#D55E00", "#CC79A7", "#F0E442"]
color_palette = sns.color_palette('bright')
# ["-", "--", "-.", ":", "-", "--", "-.",  ":", "-", "--"]
linestyle_palette = ["-", "-", "-", "-", "-", "-", "-", "-", "-", "-"]


def init_plt(full_screen=False, scale=1, use_latex=True):
    """Init the Pyplot context for scientific publication-ready figures.

    :param full_screen: If one column paper (as opposed to 2-cols),
        defaults to False
    :type full_screen: bool, optional
    :param scale: Scaling of the figure, defaults to 1
    :type scale: int, optional
    :param use_latex: To set `plt.rcParams['text.usetex']`, defaults to
        True
    :type use_latex: bool, optional
    """
    sns.set_style("ticks")
    sns.set_context("paper", font_scale=scale)
    # sns.set(font_scale=1.5, rc={'text.usetex' : False})
    plt.style.use(['science', 'ieee'])

    # most journals: >= 300dpi
    plt.rcParams["savefig.dpi"] = 600

    # most journals: 9 cm (or 3.5 inch) for single column width and 18.5 cm
    # (or 7.3 inch) for double column width.
    plt.rcParams["figure.autolayout"] = False
    if full_screen:
        plt.rcParams["figure.figsize"] = (
            width_two_cols*scale, width_one_col/2*scale)
    else:
        golden_ratio = 1.61803398875
        plt.rcParams["figure.figsize"] = (
            width_one_col*scale, width_one_col/golden_ratio*scale)

    # Font
    plt.rcParams['font.family'] = 'serif'
    plt.rcParams['font.serif'] = 'Times New Roman'  # 'Times'

    # Latex support
    plt.rc('text', usetex=True)
    plt.rcParams["text.latex.preamble"].join([
        r"\usepackage{xcolor}",
        r"\setmainfont{amsmath}",
        r"\setmainfont{amssymb}",
    ])
    # matplotlib.verbose.level = 'debug-annoying'

    # Font size
    plt.rcParams["font.size"] = 7 * scale
    plt.rcParams["axes.labelsize"] = 7 * scale
    plt.rcParams["axes.titlesize"] = 7 * scale
    plt.rcParams["xtick.labelsize"] = 5 * scale
    plt.rcParams["ytick.labelsize"] = 5 * scale
    plt.rcParams["legend.fontsize"] = 6 * scale
    plt.rcParams["legend.title_fontsize"] = 7 * scale

    # Objects size/colors
    plt.rcParams["lines.linewidth"] = .7 * scale
    plt.rcParams["lines.markersize"] = 7 * scale

    # color palette from colorbrewer
    # (up to 4 colors, good for print and black&white printing)
    # color_brewer_palette = ['#e66101', '#5e3c99', '#fdb863', '#b2abd2']

    sns.set_palette(sns.color_palette(color_palette))
    plt.rcParams['axes.prop_cycle'] = cycler(
        color=color_palette, linestyle=linestyle_palette)
    # sns.set_palette("colorblind")


def multi_format_savefig(figure, dir_name, fig_name):
    """Save a matplotlib figure to PNG, PDF, and SVG files simultaneously.

    :param figure: Matplotlib figure
    :type figure: matplotlib.figure.Figure
    :param dir_name: str
    :type dir_name: absolute path to the export directory
    :param fig_name: str
    :type fig_name: filename
    """
    if not os.path.exists(dir_name):
        print('Creating the folder ', dir_name)
        os.makedirs(dir_name, exist_ok=False)
    full_fig_path = os.path.join(dir_name, fig_name)
    figure.savefig(
        full_fig_path + ".pdf",
        format='pdf',
        dpi=300,
        bbox_inches='tight'
    )
    figure.savefig(
        full_fig_path + ".svg",
        format='svg',
        dpi=300,
        bbox_inches='tight'
    )
    figure.savefig(
        full_fig_path + ".png",
        format='png',
        dpi=300,
        bbox_inches='tight',
        transparent=True
    )
    print('   saving to ', full_fig_path + ".png + .pdf + .svg ...")


def PNG_savefig(
    figure: matplotlib.figure.Figure,
    dir_name: str,
    fig_name: str
):
    """Save a matplotlib figure to PNG.

    :param figure: Matplotlib figure
    :type figure: matplotlib.figure.Figure
    :param dir_name: str
    :type dir_name: absolute path to the export directory
    :param fig_name: str
    :type fig_name: filename
    """
    if not os.path.exists(dir_name):
        print('Creating the folder ', dir_name)
        os.makedirs(dir_name, exist_ok=False)
    figure.savefig(
        dir_name + '/' + fig_name + ".png",
        format='png',
        dpi=300,
        bbox_inches='tight',
        transparent=True
    )
    print('   saving to ', dir_name + fig_name + ".png ...")
