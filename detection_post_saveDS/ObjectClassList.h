#ifndef __OBJECTCLASSLIST_H__
#define __OBJECTCLASSLIST_H__

struct ObjectDetectionItem {
    uint8_t index;
    const char* objectName;
    uint8_t filter;
};

// List of objects the pre-trained model is capable of recognizing
// Index number is fixed and hard-coded from training
// Set the filter value to 0 to ignore any recognized objects
ObjectDetectionItem itemList[4] = {
    {0,  "neutral",         1},
    {1,  "microsueno",        1},
    {2,  "distraccion",            1},
    {3,  "uso_celular",      1},
};

#endif
