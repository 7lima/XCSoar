set size ratio -1
set key outside right

set style line 1 lt 2 lc rgb "black" lw 2
set style line 2 lt 4 lc rgb "purple" lw 1
set style line 3 lt 4 lc rgb "orange" lw 1
set style line 4 lt 4 lc rgb "blue" lw 2
set style line 5 lt 4 lc rgb "grey" lw 2
set style line 6 lt 4 lc rgb "green" lw 1
set style line 7 lt 4 lc rgb "cyan" lw 1
set style line 8 lt 2 lc rgb "red" lw 2


plot \
     'results/res-olc-solution.txt' using 1:2 with linespoints ls 8 title "olc", \
     'results/res-sample.txt' using 2:3 with lines ls 1 title "sample"
pause -1

set size noratio
set autoscale
set ylabel "h (m)"
set xlabel "t (s)"
plot \
     'results/res-olc-solution.txt' using 4:3 with linespoints ls 8 title "olc", \
     'results/res-sample.txt' using 1:4 with lines ls 1 title "sample"
pause -1

set ylabel "rank (m)"
set xlabel "t (s)"
plot \
     'results/res-olc-trace.txt' using 4:5 with lines ls 8 title "olc"
pause -1
