#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2020-2021 Smart Robotics Lab, Imperial College London, Technical University of Munich
# SPDX-FileCopyrightText: 2020-2021 Nils Funk
# SPDX-FileCopyrightText: 2020-2021 Sotiris Papatheodorou
# SPDX-License-Identifier: BSD-3-Clause

import argparse
import math
import os
import sys

import matplotlib.pyplot as plt

from typing import Dict, List, Tuple


_default_timings = "read,tracking,integration,raycast,render"
_default_plots = "timings"
_iteration_column_name = "iteration"



class QuantityValues:
    """Contains the values of some quantity for a single run."""
    def __init__(self, name: str, unit: str) -> None:
        self.name = name
        self.unit = unit
        self.values = []

    def __str__(self) -> str:
        if self.unit:
            return "{} ({})".format(self.name, self.unit)
        else:
            return "{}".format(self.name)

    def append(self, value: float) -> None:
        self.values.append(value)

    def mean(self) -> float:
        return sum(self.values) / len(self.values)



class Stats:
    """Statistics for a single run."""
    def __init__(self, filename: str) -> None:
        self.filename = filename
        # Open a file or stdin
        if not self.filename or self.filename == "-":
            f = sys.stdin
        else:
            f = open(filename)
        # Read the header and initialize the dictionaries.
        header = f.readline()
        self.stats_by_name, self.stats_by_idx = Stats._init_stats(header)
        # Read the data for each iteration.
        for line in f:
            Stats._append_stats(line, self.stats_by_idx)
        if f is not sys.stdin:
            f.close()

    def max_time(self, timings: List[str]) -> List[float]:
        # Combine the values of all specified timings in a list of lists.
        timing_values = [self.stats_by_name[t].values for t in timings]
        # Find the maximum time per iteration using the splat (*) operator.
        return [max(x) for x in zip(*timing_values)]

    def total_time(self, timings: List[str]) -> List[float]:
        # Combine the values of all specified timings in a list of lists.
        timing_values = [self.stats_by_name[t].values for t in timings]
        # Sum the timing values to get the total frame time per iteration using
        # the splat (*) operator.
        return [sum(x) for x in zip(*timing_values)]

    @staticmethod
    def _init_stats(header: str) -> Tuple[Dict[str,QuantityValues],Dict[int,QuantityValues]]:
        """Given the TSV header return two dictionaries to access the individual
        quantities by name or zero-based column index."""
        stats_by_name = {}
        stats_by_idx = {}
        columns = header.strip().split("\t")
        for i, c in enumerate(columns):
            # Extract the name and its unit.
            unit_idx = c.rfind(" [")
            if unit_idx >= 0:
                name = c[0:unit_idx].strip()
                unit = c[unit_idx+2:-1].strip()
            else:
                name = c.strip()
                unit = ""
            # Set the unit to empty if it's not a unit.
            if unit in ["#", "-"]:
                unit = ""
            stats_by_name[name] = QuantityValues(name, unit)
            # The following will create a reference, not a copy.
            stats_by_idx[i] = stats_by_name[name]
        return stats_by_name, stats_by_idx

    @staticmethod
    def _append_stats(line: str, stats_by_idx: Dict[int,QuantityValues]) -> None:
        columns = line.strip().split("\t")
        for i, c in enumerate(columns):
            if c != "*":
                stats_by_idx[i].append(float(c))



class Plotter:
    """Creates plots from statistics for multiple runs."""
    plot_names = ["timings", "path"]
    plot_names_str = ", ".join(plot_names)

    def __init__(self, stats: List[Stats], args) -> None:
        # Create dictionaries from plot names to class methods.
        self._plots_to_plot_funcs = dict(zip(Plotter.plot_names,
            [self._plot_timings, self._plot_path]))
        self._plots_to_limit_funcs = dict(zip(Plotter.plot_names,
            [self._limits_timings, self._limits_path]))

        self.stats = stats
        self.plots = args.plots
        self.timings = args.timings
        self.lineplot = args.lineplot
        self.save = args.save
        self.quiet = args.quiet

        # Compute the axis limits for all plots.
        self._limits = {plot_name: self._plots_to_limit_funcs[plot_name]() for plot_name in self.plots}

        # Create a figure for each run.
        for s in self.stats:
            self._plot(s)
            if self.save:
                # Generate the filename of the PNG file.
                if not s.filename or s.filename == "-":
                    filename_png = "figure.png"
                else:
                    filename_png = s.filename + ".png"
                figure = plt.gcf()
                figure.set_size_inches(16, 9)
                plt.savefig(filename_png, dpi = 200, bbox_inches="tight")
        if not self.quiet:
            plt.show()

    def _limits_timings(self) -> List[float]:
        x_min = min([s.stats_by_name[_iteration_column_name].values[0] for s in self.stats])
        x_max = max([s.stats_by_name[_iteration_column_name].values[-1] for s in self.stats])
        if self.lineplot:
            y_max = max([max(s.max_time(self.timings)) for s in self.stats])
        else:
            y_max = max([max(s.total_time(self.timings)) for s in self.stats])
        return [x_min, x_max, 0, y_max]

    def _limits_path(self) -> List[float]:
        x_min = min([min(s.stats_by_name["tx"].values) for s in self.stats])
        x_max = max([max(s.stats_by_name["tx"].values) for s in self.stats])
        y_min = min([min(s.stats_by_name["ty"].values) for s in self.stats])
        y_max = max([max(s.stats_by_name["ty"].values) for s in self.stats])
        return [x_min, x_max, y_min, y_max]

    def _plot_timings(self, stats: Stats, axis) -> None:
        # ASSUMPTION: the time units are seconds.
        axis.set_title("Computation time")
        axis.set_xlabel("Frame")
        axis.set_ylabel("Time (ms)")
        iterations = stats.stats_by_name[_iteration_column_name].values
        # Stack all the timing data together.
        timing_data = [[1000 * x for x in stats.stats_by_name[t].values] for t in self.timings]
        if self.lineplot:
            for i, t in enumerate(self.timings):
                axis.plot(iterations, timing_data[i], label=t)
        else:
            axis.stackplot(iterations, timing_data, labels=self.timings)
        axis.grid(True)
        axis.legend(loc="upper left")
        l = self._limits["timings"]
        axis.set_xlim(l[0], l[1])
        axis.set_ylim(l[2], 1000 * l[3])

    def _plot_path(self, stats: Stats, axis) -> None:
        axis.set_title("Path")
        axis.set_xlabel("x (m)")
        axis.set_ylabel("y (m)")
        axis.plot(stats.stats_by_name["tx"].values,
                stats.stats_by_name["ty"].values)
        axis.plot(stats.stats_by_name["tx"].values[0],
                stats.stats_by_name["ty"].values[0], color="blue", marker="o",
                linestyle="", label="Start")
        axis.plot(stats.stats_by_name["tx"].values[-1],
                stats.stats_by_name["ty"].values[-1], color="blue", marker="x",
                linestyle="", label="End")
        axis.grid(True)
        axis.legend(loc="upper left")
        l = self._limits["path"]
        axis.set_xlim(l[0], l[1])
        axis.set_ylim(l[2], l[3])

    def _plot(self, stats: Stats) -> None:
        fig = plt.figure()
        fig.canvas.set_window_title(os.path.basename(stats.filename))

        num_plots = len(self.plots)
        for i, plot_name in enumerate(self.plots):
            ax = fig.add_subplot(num_plots, 1, i + 1)
            self._plots_to_plot_funcs[plot_name](stats, ax)



def parse_arguments():
    parser = argparse.ArgumentParser(description=(""))
    parser.add_argument("files", nargs="*", metavar="FILE", default=["-"],
            help=("""A TSV file containing the supereight log. With no FILE or
            when FILE is -, read standard input. If multiple files are provided
            each one will be plotted in a different window using the same axis
            limits for easy comparison."""))
    parser.add_argument("-t", "--timings", metavar="NAMES",
            default=_default_timings,
            help="""select which timings to show. NAMES is a comma-separated
            list containing any TSV column name (default: """ + _default_timings
            + ")")
    parser.add_argument("-p", "--plots", metavar="NAMES",
            default=_default_plots,
            help="""select which plots to show. NAMES is a comma-separated list
            which can contain any of the names: """ + Plotter.plot_names_str + """
            (default: """ + _default_plots + ")")
    parser.add_argument("-l", "--lineplot", action="store_true",
            help="""create a line plot instead of a stacked plot for timings""")
    parser.add_argument("-s", "--save", action="store_true",
            help="""save the plots using the same filenames appended with the
            .png extension""")
    parser.add_argument("-q", "--quiet", action="store_true",
            help="""hide all plots""")
    args = parser.parse_args()
    # Extract the timing names.
    args.timings = args.timings.split(",")
    # Extract the plot names and ensure they are valid.
    args.plots = args.plots.split(",")
    for plot_name in args.plots:
        if plot_name not in Plotter.plot_names:
            print("Error, invalid plot name: " + plot_name)
            print("Expected one of: " + Plotter.plot_names_str)
            sys.exit(2)
    return args



if __name__ == "__main__":
    try:
        args = parse_arguments()
        stats = [Stats(filename) for filename in args.files]
        plotter = Plotter(stats, args)
    except (KeyboardInterrupt, BrokenPipeError):
        pass

