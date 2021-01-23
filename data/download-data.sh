#!/bin/bash
BASEDIR=$(dirname $0)
echo "Script location: ${BASEDIR}"

wget -c -P $BASEDIR http://deepscene.cs.uni-freiburg.de/static/datasets/freiburg_forest_multispectral_annotated.tar.gz

echo "Extracting data..."
tar -xzf $BASEDIR/freiburg_forest_multispectral_annotated.tar.gz --strip 1 -C $BASEDIR 

echo "Deleting tarbell..."
rm $BASEDIR/freiburg_forest_multispectral_annotated.tar.gz
