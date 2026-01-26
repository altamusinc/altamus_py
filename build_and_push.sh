source token.env # holds the TWINE_TOKEN variable

rm -rf dist
rm -rf build
python -m build
twine upload dist/* --username __token__ --password $TWINE_TOKEN --verbose