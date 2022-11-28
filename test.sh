cd Build || exit 1;

chmod +x Test/unit_test;

ctest --output-on-failure;
