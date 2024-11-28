echo "Installing Pixi"
curl -fsSL https://pixi.sh/install.sh | bash

echo "Running colcon build"
/home/runner/.pixi/bin/pixi run colcon build

if [ $? -eq 0 ] ; then
  echo "Pass: Build succesful"
else
  echo "Fail: Errors in build"
  exit 1
fi

echo "Running colcon test"
/home/runner/.pixi/bin/pixi run colcon test --ctest-args rp_simulator_test

if [ $? -eq 0 ] ; then
  echo "Pass: test ok"
else
  echo "Fail: errors in test"
  exit 1
fi

output=$(/home/runner/.pixi/bin/pixi run colcon test-result --all)
if echo "$output" | grep "0 failure" ; then
  echo "Pass: Test output is correct"
else
  echo "Fail: Test output is incorrect"
  exit 1
fi

exit 0 