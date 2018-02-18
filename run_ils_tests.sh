#!/bin/bash

# Base query setup
runs=5
iterations=20
query="http://localhost:8989/route?point=43.009327%2C-74.009166&point=43.009327%2C-74.009166\
&vehicle=racingbike\
&weighting=shortest\
&seed=1518916497727\
&max_iterations=$iterations\
&run_tests=true\
&num_runs=$runs\
&max_cost=40000"

echo "LS Algorithm (Normal)"
curl -s -o /dev/null "$query\
&algorithm=ils-ls\
&output_file=ls_normal.csv\
&mode=0"

echo "LS Algorithm (No Minimum Score)"
curl -s -o /dev/null "$query\
&algorithm=ils-ls\
&output_file=ls_no_min_score.csv\
&min_road_score=0\
&mode=0"

echo "LS Algorithm (No Minimum Length)"
curl -s -o /dev/null "$query\
&output_file=ls_no_min_length.csv\
&algorithm=ils-ls\
&min_road_length=0\
&mode=0"	

echo "LS Algorithm (No Minimums)"
curl -s -o /dev/null "$query\
&algorithm=ils-ls\
&output_file=ls_no_mins.csv\
&min_road_score=0\
&min_road_length=0\
&mode=0"

echo "LS Algorithm (Fixed Budget Percentage)"
curl -s -o /dev/null "$query\
&algorithm=ils-ls\
&output_file=ls_fixed_budget_percent.csv\
&min_road_score=0\
&min_road_length=0\
&mode=1\
&budget_percentage=0.5"

echo "LS Algorithm (Incremental Budget)"
curl -s -o /dev/null "$query\
&algorithm=ils-ls\
&output_file=ls_incremental_budget.csv\
&min_road_score=0\
&min_road_length=0\
&mode=2\
&budget_percentage=0.5"

echo "LS Algorithm (Incremental Budget 2)"
curl -s -o /dev/null "$query\
&algorithm=ils-ls\
&output_file=ls_incremental_budget2.csv\
&min_road_score=0\
&min_road_length=0\
&mode=2\
&budget_percentage=0"

echo "LS Algorithm (Normalized Scores)"
curl -s -o /dev/null "$query\
&algorithm=ils-ls\
&output_file=ls_normalized_scores.csv\
&min_road_score=0\
&min_road_length=0\
&mode=3\
&score_cutoff=0.5"

echo
echo "Test queries complete!"