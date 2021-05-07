#include <iostream>
#include <thread>
#include <chrono>
#include <cmath>

#include <gtest/gtest.h>

class Calc
{
public:
    static int add(int x, int y) { return x + y; }
    static int sub(int x, int y) { return x - y; }
};

class CalcTest : public ::testing::Test
{
public:

    CalcTest()
    {
        std::cout << "CalcTest()" << std::endl;
    }

    virtual void SetUp()
    {
        std::cout << "SetUp()" << std::endl;
    }

    virtual void TearDown()
    {
        std::cout << "TearDown()" << std::endl;
    }

    ~CalcTest()
    {
        std::cout << "~CalcTest()" << std::endl;
    }
};

TEST_F(CalcTest, Add) {
    ASSERT_EQ(2, Calc::add(1, 1));
    ASSERT_EQ(5, Calc::add(3, 2));
    ASSERT_EQ(10, Calc::add(7, 3));
}

TEST_F(CalcTest, Sub) {
    ASSERT_EQ(3, Calc::sub(5, 2));
    ASSERT_EQ(-10, Calc::sub(5, 15));
}

TEST(BasicTest, testWillFail)
{
    ASSERT_EQ(42, 0);
}

TEST(SquareRootTest, PositiveNos) { 
    EXPECT_DOUBLE_EQ(18.0, sqrt(324.0));
    EXPECT_DOUBLE_EQ(25.4, sqrt(645.16));
    EXPECT_DOUBLE_EQ(50.332, sqrt(2533.310224));
}

TEST(module_name, test_name) {
    std::cout << "Hello world!" << std::endl;
    ASSERT_EQ(1+1, 2);
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}