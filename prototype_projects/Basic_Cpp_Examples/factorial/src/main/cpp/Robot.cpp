// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <numbers>
#include <iostream>

int factorial(int number)
{
  int answer = number;
  int loop;

  for(loop = (number - 1); loop >= 2; loop -= 1)
  {
      answer = answer * loop;
  }

  return answer;
}

int main()
{
  int input;
  int final;  

  std::cout << "[Factorial Example Starting]" << std::endl;

  do 
  {
    std::cout << "Enter a number (0 to end): ";
    std::cin >> input;

    if(input == 0)
    {
      std::cout << "Good Bye World" << std::endl; 
      break;
    }
    else
    {
      if (input < 0)
      {
        std::cout << "Factorial expects a positive number!!!!" << std::endl;
        continue;
      }
    }

    final = factorial(input);

    std::cout << input << " factorial = " << final << std::endl;
  } while (true);

  return 0;
}
