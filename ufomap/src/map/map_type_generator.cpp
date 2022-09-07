/*!
 * UFOMap: An Efficient Probabilistic 3D Mapping Framework That Embraces the Unknown
 *
 * @author Daniel Duberg (dduberg@kth.se)
 * @see https://github.com/UnknownFreeOccupied/ufomap
 * @version 1.0
 * @date 2022-05-13
 *
 * @copyright Copyright (c) 2022, Daniel Duberg, KTH Royal Institute of Technology
 *
 * BSD 3-Clause License
 *
 * Copyright (c) 2022, Daniel Duberg, KTH Royal Institute of Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *     list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

// STL
#include <cstddef>
#include <filesystem>
#include <iostream>
#include <regex>
#include <string>
#include <utility>
#include <vector>

struct Attribute {
	std::string name;
	std::string type;
};

using Attributes = std::vector<Attribute>;

void printWelcome() { std::cout << "Hi and welcome to the UFOMap map type generator\n"; }

void printHelp() { std::cout << "You should know this!\n"; }

bool isUFOMapPath(std::filesystem::path const& path)
{
	if (!std::filesystem::is_directory(path)) {
		std::cout << "'" << path << "' does not exist\n";
		return false;
	}
	return true;
}

std::filesystem::path UFOMapPath()
{
	std::filesystem::path path;
	do {
		std::cout << "Where is UFOMap located: ";
		std::string temp;
		std::getline(std::cin, temp);
		path = temp;
	} while (!isUFOMapPath(path));
	return path;
}

bool isValidMapTypeName(std::string const& name)
{
	// TODO: Implement

	// TODO: Check if already exists

	return true;
}

std::string mapTypeName()
{
	std::string name;
	do {
		std::cout << "Map type name: ";
		std::getline(std::cin, name);
	} while (!isValidMapTypeName(name));
	return name;
}

std::size_t numAttributes()
{
	while (true) {
		std::cout << "Number of attributes for map type: ";
		std::string temp;
		std::getline(std::cin, temp);
		try {
			return std::stoi(temp);
		} catch (std::invalid_argument const& ex) {
			std::cout << "std::invalid_argument::what(): " << ex.what() << '\n';
		} catch (std::out_of_range const& ex) {
			std::cout << "std::out_of_range::what(): " << ex.what() << '\n';
		}
	}
}

bool isValidAttribute(Attribute const& attribute)
{
	// TODO: Implement
	return true;
}

Attributes mapAttributes()
{
	auto const num = numAttributes();
	Attributes attributes;
	attributes.reserve(num);
	for (std::size_t i = 0; num != i; ++i) {
		Attribute attribute;
		do {
			std::cout << "Attribute " << i << " name: ";
			std::getline(std::cin, attribute.name);
			std::cout << "Attribute " << i << " type: ";
			std::getline(std::cin, attribute.type);
		} while (!isValidAttribute(attribute));
		attributes.push_back(std::move(attribute));
	}
	return attributes;
}

void generateMapType(std::filesystem::path const& path, std::string const& name,
                     Attributes const& attributes)
{
	std::cout << "Generating map type\n";

	// TODO: Implement

	std::cout << "Map type has been generated\n";
}

void printGoodBye(std::filesystem::path const& path, std::string const& name,
                  Attributes const& attributes)
{
	// TODO: Implement
}

int main(int argc, char* argv[])
{
	printWelcome();

	auto path = UFOMapPath();
	auto name = mapTypeName();
	auto attributes = mapAttributes();

	generateMapType(path, name, attributes);

	printGoodbye();

	return 0;
}