echo "Running test queries!"
curl -s -o /dev/null "http://localhost:8989/route?point=43.009449%2C-74.006824&point=43.009417%2C-74.007511&locale=en-US&vehicle=racingbike&weighting=shortest&elevation=false&algorithm=ils&instructions=false&use_miles=false&layer=Omniscale&output_file=data1.csv"
curl -s -o /dev/null "http://localhost:8989/route?point=43.009449%2C-74.006824&point=43.009417%2C-74.007511&locale=en-US&vehicle=racingbike&weighting=shortest&elevation=false&algorithm=ils&instructions=false&use_miles=false&layer=Omniscale&output_file=data2.csv"
curl -s -o /dev/null "http://localhost:8989/route?point=43.009449%2C-74.006824&point=43.009417%2C-74.007511&locale=en-US&vehicle=racingbike&weighting=shortest&elevation=false&algorithm=ils&instructions=false&use_miles=false&layer=Omniscale&output_file=data3.csv"
echo "Test queries complete!"