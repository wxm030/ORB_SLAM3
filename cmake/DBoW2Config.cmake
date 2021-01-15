find_library(DBoW2_LIBRARY DBoW2
    PATHS "/usr/local/lib"
)
find_path(DBoW2_INCLUDE_DIR DBoW2/BowVector.h DBoW2/FeatureVector.h
    PATHS "/usr/local/include"
)
set(DBoW2_LIBS ${DBoW2_LIBRARY})
set(DBoW2_LIBRARIES ${DBoW2_LIBRARY})
set(DBoW2_INCLUDE_DIRS ${DBoW2_INCLUDE_DIR})
