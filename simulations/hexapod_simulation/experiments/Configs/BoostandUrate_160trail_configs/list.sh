for trail in {1..160}
do
  echo TRIAL $trail/160
  f=config_trail$trail.txt
  echo $f
  cat $f
done
