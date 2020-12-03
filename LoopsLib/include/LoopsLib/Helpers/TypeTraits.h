#ifndef LOOPSLIB_HELPERS_TYPETRAITS_H
#define LOOPSLIB_HELPERS_TYPETRAITS_H
#include <tuple>
#include <functional>
namespace LoopsLib::Helpers
{
    template <typename T> struct LambdaArgs : LambdaArgs<decltype(&T::operator())> {};

    template <typename C, typename ... Args>
    struct LambdaArgs<void(C::*)(Args...) const>
    {
        using type = void(Args...);
        using ArgTuple = std::tuple<Args...>;
        using ReturnType = void;
        using FuncType = std::function<void(Args...)>;
    };
    template <typename C, typename RetType, typename ... Args>
    struct LambdaArgs<RetType(C::*)(Args...) const>
    {
        using type = RetType(Args...);
        using ArgTuple = std::tuple<Args...>;
        using ReturnType = RetType;
        using FuncType = std::function<RetType(Args...)>;
    };

    template<typename _Type>
    struct TypeHolder
    {
        using Type = _Type;
    };

    struct IdGenerator{
        static long long getId(){
            static long long id = 0;
            return ++id;
        }
    };
    
    template<typename T>
    struct IdForType{
        static long long getId(){
            static long long id = IdGenerator::getId();
            return id;
        }
    };
}
#endif