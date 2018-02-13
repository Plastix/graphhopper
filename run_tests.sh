#!/bin/bash

# Base query setup
runs=3
iterations=20
query="http://localhost:8989/route?point=43.009449%2C-74.006824&point=43.009417%2C-74.007511\
&vehicle=racingbike\
&weighting=shortest\
&algorithm=ils\
&max_iterations=$iterations\
&num_runs=$runs\
&max_cost=40000"

echo "LS Algorithm (Normal)"
curl -s -o /dev/null "$query\
&output_file=ls_normal.csv\
&mode=0"

echo "LS Algorithm (No Minimum Score)"
curl -s -o /dev/null "$query\
&output_file=ls_no_min_score.csv\
&min_road_score=0\
&mode=0"

echo "LS Algorithm (No Minimum Length)"
curl -s -o /dev/null "$query\
&output_file=ls_no_min_length.csv\
&min_road_length=0\
&mode=0"

echo "LS Algorithm (No Minimums)"
curl -s -o /dev/null "$query\
&output_file=ls_no_mins.csv\
&min_road_score=0\
&min_road_length=0\
&mode=0"

echo "LS Algorithm (Fixed Budget Percentage)"
curl -s -o /dev/null "$query\
&output_file=ls_fixed_budget_percent.csv\
&min_road_score=0\
&min_road_length=0\
&mode=1\
&budget_percentage=0.5"

echo "LS Algorithm (Incremental Budget)"
curl -s -o /dev/null "$query\
&output_file=ls_incremental_budget.csv\
&min_road_score=0\
&min_road_length=0\
&mode=2\
&budget_percentage=0.5"

echo "LS Algorithm (Incremental Budget 2)"
curl -s -o /dev/null "$query\
&output_file=ls_incremental_budget2.csv\
&min_road_score=0\
&min_road_length=0\
&mode=2\
&budget_percentage=0"

echo "LS Algorithm (Normalized Scores)"
curl -s -o /dev/null "$query\
&output_file=ls_normalized_scores.csv\
&min_road_score=0\
&min_road_length=0\
&mode=3\
&score_cutoff=0.5"

echo "LS Algorithm (Normalized Scores 2)"
curl -s -o /dev/null "$query\
&output_file=ls_normalized_scores2.csv\
&min_road_score=0\
&min_road_length=0\
&mode=3\
&score_cutoff=0.8"

echo
echo "Test queries complete!"