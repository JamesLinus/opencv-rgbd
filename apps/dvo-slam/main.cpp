#include <dvo/dense_tracking.h>
#include <dvo/util/revertable.h>

using namespace dvo;
using namespace dvo::core;
using namespace dvo::util;

int main()
{
    IntrinsicMatrix intrinsics = IntrinsicMatrix::create(520.9f, 521.0f, 325.1f, 249.7f);//fr2

    dvo::DenseTracker denseTracker;
    //denseTracker.configure();

	return 0;
}