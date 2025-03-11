# Supereight common issues

This file contains common issues and problematic code patterns that might be
encountered while developing supereight.



## OpenMP parallel for loops

``` cpp
// Correct, loop variable declared inside the loop
#pragma omp parallel for
for (int i = 0; i < 8; i++)

// Wrong, loop variable declared outside the loop
int i;
#pragma omp parallel for
for (i = 0; i < 8; i++)
```



## Initialization of smart pointers

Alignment issues can arise when initializing `std::shared_ptr`/`std::unique_ptr`
to objects containing Eigen members.

``` cpp
// Correct, using the std::shared_ptr/std::unique_ptr constructor
x = std::shared_ptr<T>(new T(constructor_parameter));

// Wrong, using std::make_shared/std::make_unique
x = std::make_shared<T>(constructor_parameter);
```

Most C++ implementations seem to ignore the overloaded operator new defined by
Eigen. See [here](https://gitlab.com/libeigen/eigen/-/issues/1049) for details.



## Using `std::shared_ptr` in loops

Creating copies of `std::shared_ptr`s in loops can significantly affect
performance. Even if the shared pointer is only part of an expression and not
stored it is still created and destroyed.

``` cpp
// Correct, create a single copy of the shared pointer
int size = map.getOctree().getSize();
for (...) {
    // Do stuff
}

// Wrong, create one copy of the shared pointer per iteration
for (...) {
    int size = map.getOctree().getSize();
    // Do stuff
}
```

