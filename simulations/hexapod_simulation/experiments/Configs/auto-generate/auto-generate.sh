for trail in {1..100}
do
  cp config_trail.txt config_trail$trail.txt
  echo l2_Time = $trail >> config_trail$trail.txt
done


# l2_Time = 1
