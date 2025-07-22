/*
© Siemens AG, 2025
Author: Mehmet Emre Cakal (emre.cakal@siemens.com)

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at
<http://www.apache.org/licenses/LICENSE-2.0>.
Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

Note: This file is a refactored version of the original MessageParser.cs
    written by Sifan Ye, © Siemens AG, 2019
*/

using System;
using System.IO;
using System.Collections.Generic;
using System.Linq;

namespace RosSharp.RosBridgeClient.MessageGeneration
{
    public class MessageParser
    {

        private List<MessageToken> tokens;

        private readonly string inFilePath;
        private readonly string rosPackageName;
        private readonly string className;
        private readonly string rosMsgName;
        private readonly string type;
        private readonly string outPath;
        private string outFilePath;

        private HashSet<string> imports = new HashSet<string>();

        private readonly Dictionary<string, string> builtInTypeMapping;
        private readonly Dictionary<string, string> builtInTypesDefaultInitialValues;

        private uint lineNum = 1;

        private Dictionary<string, string> symbolTable = new Dictionary<string, string>();
        private HashSet<string> constantValuedIdentifiers = new HashSet<string>();
        private HashSet<string> defaultValuedIdentifiers = new HashSet<string>();

        private Dictionary<string, int> arraySizes = new Dictionary<string, int>();
        private Dictionary<string, string> boundedArraySizeDirection = new Dictionary<string, string>();

        private string body = "";

        public List<string> warnings = new List<string>();
        protected bool isRos2;

        private readonly Dictionary<string, string> defaultValues = new Dictionary<string, string>();

        public MessageParser(List<MessageToken> tokens, string outPath, string rosPackageName, string type, Dictionary<string, string> builtInTypeMapping, Dictionary<string, string> builtInTypesDefaultInitialValues, string className = "", string rosMsgName = "", bool isRos2 = true)
        {
            this.inFilePath = tokens[0].content;
            // this.tokens = tokens;
            this.tokens = tokens.Skip(1).ToList();
            this.rosPackageName = rosPackageName;
            this.type = type;
            this.builtInTypeMapping = builtInTypeMapping;
            this.builtInTypesDefaultInitialValues = builtInTypesDefaultInitialValues;
            this.isRos2 = isRos2;

            this.className = string.IsNullOrWhiteSpace(className) ?
                MsgAutoGenUtilities.CapitalizeFirstLetter(Path.GetFileNameWithoutExtension(inFilePath)) :
                className;

            this.rosMsgName = string.IsNullOrWhiteSpace(rosMsgName) ?
                MsgAutoGenUtilities.CapitalizeFirstLetter(Path.GetFileNameWithoutExtension(inFilePath)) :
                rosMsgName;

            this.outPath = outPath;
            this.outFilePath = Path.Combine(outPath, type);


        }
        #region MAIN_PARSE_LOGIC
        public void Parse()
        {
            // Create output directory if it does not exist
            Directory.CreateDirectory(outFilePath);
            this.outFilePath = Path.Combine(this.outFilePath, this.className + ".cs");

            using (StreamWriter writer = new StreamWriter(outFilePath, false))
            {
                writer.Write(MsgAutoGenUtilities.BLOCK_COMMENT + "\n");

                body = ReadTokens();

                // Write ROS version preprocessor directive
                writer.Write(isRos2 ? "#if ROS2" : "#if !ROS2");

                // Write imports
                writer.Write(MsgAutoGenUtilities.GenerateImports(imports));

                // Write namespace
                writer.Write(
                    "namespace RosSharp.RosBridgeClient.MessageTypes." + MsgAutoGenUtilities.ResolvePackageName(rosPackageName) + "\n" +
                    "{\n");

                // Write class declaration
                writer.Write(MsgAutoGenUtilities.ONE_TAB + $"public class {className} : Message\n" + MsgAutoGenUtilities.ONE_TAB + "{\n");

                // Write ROS package name
                writer.Write(isRos2 ?
                    MsgAutoGenUtilities.TWO_TABS + $"public const string RosMessageName = \"{rosPackageName}/{type}/{rosMsgName}\";\n\n" :
                    MsgAutoGenUtilities.TWO_TABS + $"public const string RosMessageName = \"{rosPackageName}/{rosMsgName}\";\n\n");

                // Write body
                writer.Write($"{body}\n");

                // Write constructors
                writer.Write(GenerateDefaultValueConstructor());
                if (symbolTable.Count != 0 && symbolTable.Keys.Except(constantValuedIdentifiers).Any())
                    writer.Write("\n" + GenerateParameterizedConstructor());

                // Close class, namespace, and preprocessor directive
                writer.Write(MsgAutoGenUtilities.ONE_TAB + "}\n");
                writer.Write("}\n");
                writer.Write("#endif\n");

                writer.Flush();
                writer.Close();
            }
        }
        #endregion
        #region DECLARATIONS
        private string ReadTokens()
        {
            string body = "";
            for (int tokenIndex = 0; tokenIndex < tokens.Count; tokenIndex++)
            {
                var token = tokens[tokenIndex];
                var tokenType = token.type;
                if (tokenType == MessageTokenType.Comment)
                {
                    body += $"{MsgAutoGenUtilities.TWO_TABS}// {token.content}\n";
                }
                else if (tokenType == MessageTokenType.BuiltInType ||
                         tokenType == MessageTokenType.DefinedType ||
                         tokenType == MessageTokenType.Header)
                {
                    body += ReadLine(ref tokenIndex);
                }
                else
                {
                    throw new MessageParserException(
                        "Unexpected token '" + token.content +
                        "' at " + inFilePath + ":" + token.lineNum +
                        ". Expecting a comment or field declaration.");
                }
            }
            return body;
        }
        private string ReadLine(ref int tokenIndex)
        {
            var token = tokens[tokenIndex];
            bool canHaveConstDecl = false;
            string declaration = $"{MsgAutoGenUtilities.TWO_TABS}public ";
            string resolvedType = MsgAutoGenUtilities.ResolveType(token, ref canHaveConstDecl, ref imports, builtInTypeMapping);
            bool isArray = false;
            int arraySize = 0;
            string boundDir = "";

            // --- Array Declaration ---
            if (tokenIndex + 1 < tokens.Count)
            {
                var nextToken = tokens[tokenIndex + 1];

                if (nextToken.type == MessageTokenType.FixedSizeArray ||
                    nextToken.type == MessageTokenType.VariableSizeArray ||
                    nextToken.type == MessageTokenType.BoundedVariableSizeArray)
                {
                    isArray = true;
                    tokenIndex++;
                    if (nextToken.type == MessageTokenType.FixedSizeArray)
                        arraySize = int.Parse(nextToken.content);

                    else if (nextToken.type == MessageTokenType.BoundedVariableSizeArray)
                    {
                        var parts = nextToken.content.Split('~');
                        boundDir = parts[0];
                        arraySize = int.Parse(parts[1]);
                    }
                    resolvedType += "[]";
                }
            }

            // --- Identifier Declaration ---
            string identifier = MsgAutoGenUtilities.CheckAndCorrectIdentifier(tokens[++tokenIndex].content, builtInTypeMapping, symbolTable, inFilePath, lineNum, ref warnings);
            symbolTable[identifier] = resolvedType;

            if (isArray)
            {
                arraySizes[identifier] = arraySize;
                if (!string.IsNullOrEmpty(boundDir))
                    boundedArraySizeDirection[identifier] = boundDir;
            }

            // --- Value Declaration ---
            if (tokenIndex + 1 < tokens.Count)
            {
                var nextToken = tokens[tokenIndex + 1];
                // Constant
                if (nextToken.type == MessageTokenType.ConstantValueDeclaration)
                {
                    if (canHaveConstDecl)
                    {
                        declaration += "const " + resolvedType + " " + identifier + " = " +
                            ParseAnyValue(resolvedType, nextToken.content, identifier);
                        constantValuedIdentifiers.Add(identifier);
                        tokenIndex++;
                    }
                    else
                    {
                        throw new MessageParserException(
                            "Type " + resolvedType +
                            "' at " + inFilePath + ":" + nextToken.lineNum +
                            " cannot have constant declaration");
                    }
                }
                // Default
                else if (nextToken.type == MessageTokenType.DefaultValueDeclaration)
                {
                    var value = ParseAnyValue(resolvedType, nextToken.content, identifier);
                    defaultValuedIdentifiers.Add(identifier);
                    defaultValues[identifier] = value;

                    // If bounded array, encapsulate with backing field
                    declaration += !string.IsNullOrEmpty(boundDir) ?
                        MsgAutoGenUtilities.BoundedArrayEncapsulationBackingField(resolvedType, arraySize, boundDir, identifier, value) :
                        $"{resolvedType} {identifier} = {value}";

                    tokenIndex++;
                }
                // No value declared
                else
                {
                    if (!string.IsNullOrEmpty(boundDir))
                        declaration += $"{MsgAutoGenUtilities.BoundedArrayEncapsulationBackingField(resolvedType, arraySize, boundDir, identifier)}";

                    else
                        declaration += $"{resolvedType} {identifier}{MsgAutoGenUtilities.PROPERTY_EXTENSION}\n";
                }
            }
            // Last token: No value declaration, just type and identifier.
            else
            {
                declaration += resolvedType + " " + identifier + MsgAutoGenUtilities.PROPERTY_EXTENSION + "\n";
            }
            return declaration;
        }

        #endregion
        #region VALUE_DECLARATION
        private string ParseAnyValue(string type, string declaration, string identifier)
        {
            string formattedValue;

            // Check for comment, if there is one: split it and keep the value part.
            var commentSplit = declaration.Split('#', 2);
            var valueContent = commentSplit[0].Trim();
            var comment = commentSplit.Length > 1 ? $"//  {commentSplit[1]}" : "";

            // Check if this is a bounded array
            if (declaration.Contains("["))
            {
                string baseType = type.EndsWith("]") ? type[..^2] : type;
                var parsedElements = valueContent.Trim('[', ']').Split(',').Select(v => ParseSingleValue(baseType, v.Trim()));
                var arraySize = arraySizes[identifier] > 0 ? arraySizes[identifier] : -1; // -1 = variable size array

                int count = parsedElements.Count();
                if (boundedArraySizeDirection != null && boundedArraySizeDirection.TryGetValue(identifier, out var direction))
                {
                    if ((direction == ">=" && count < arraySize) || (direction == "<=" && count > arraySize))
                        throw new MessageParserException($"Bounded array '{identifier}' invalid size: requires {direction} {arraySize}, got {count}");

                    // For bounded arrays, use the actual element count for initialization
                    arraySize = count;
                }
                else if (arraySize > 0 && count != arraySize)
                {
                    throw new MessageParserException($"Array initializer size mismatch: expected {arraySize}, got {count}");
                }

                var sizeSpecifier = arraySize == -1 ? "" : arraySize.ToString();
                formattedValue = $"new {baseType}[{sizeSpecifier}] {{ {string.Join(", ", parsedElements)} }}";
            }
            // Otherwise single value
            else
            {
                formattedValue = ParseSingleValue(type, valueContent);
            }

            return $"{formattedValue};{comment}\n";
        }
        private string ParseSingleValue(string type, string value)
        {
            value = value.Trim();
            return type switch
            {
                "bool" =>
                    (value.ToLower() == "true" || value.ToLower() == "1") ? "true" :
                    (value.ToLower() == "false" || value.ToLower() == "0") ? "false" :
                    throw new MessageParserException($"Invalid bool value: {value}"),
                "byte" => byte.TryParse(value, out _) ? value : throw new MessageParserException($"Invalid value: {value}"),
                "sbyte" => sbyte.TryParse(value, out _) ? value : throw new MessageParserException($"Invalid value: {value}"),
                "short" => short.TryParse(value, out _) ? value : throw new MessageParserException($"Invalid value: {value}"),
                "ushort" => ushort.TryParse(value, out _) ? value : throw new MessageParserException($"Invalid value: {value}"),
                "int" => int.TryParse(value, out _) ? value : throw new MessageParserException($"Invalid value: {value}"),
                "uint" => uint.TryParse(value, out _) ? value : throw new MessageParserException($"Invalid value: {value}"),
                "long" => long.TryParse(value, out _) ? value : throw new MessageParserException($"Invalid value: {value}"),
                "ulong" => ulong.TryParse(value, out _) ? value : throw new MessageParserException($"Invalid value: {value}"),
                "float" => float.TryParse(value, out _) ? value : throw new MessageParserException($"Invalid value: {value}"),
                "double" => double.TryParse(value, out _) ? value : throw new MessageParserException($"Invalid value: {value}"),
                "string" => $"\"{value.Trim('"')}\"",
                _ => throw new MessageParserException($"Unsupported or invalid constant type: {type}")
            };
        }
        #endregion
        #region CONSTRUCTORS
        private string GenerateDefaultValueConstructor()
        {
            string constructor = $"{MsgAutoGenUtilities.TWO_TABS}public {className}()\n"
                               + $"{MsgAutoGenUtilities.TWO_TABS}{{\n";

            foreach (string identifier in symbolTable.Keys)
            {
                if (constantValuedIdentifiers.Contains(identifier) || defaultValuedIdentifiers.Contains(identifier)) continue;

                constructor += $"{MsgAutoGenUtilities.TWO_TABS}{MsgAutoGenUtilities.ONE_TAB}this.{identifier} = ";
                string type = symbolTable[identifier];

                if (builtInTypesDefaultInitialValues.TryGetValue(type, out var defaultValue))
                {
                    constructor += defaultValue;
                }
                else if (arraySizes.TryGetValue(identifier, out var arraySize))
                {
                    constructor += $"new {type[..^1]}{arraySize}]";
                }
                else
                {
                    constructor += $"new {type}()";
                }
                constructor += ";\n";
            }
            return constructor + $"{MsgAutoGenUtilities.TWO_TABS}}}\n";
        }
        private string GenerateParameterizedConstructor()
        {
            string constructor = "";
            string parameters = "";
            string assignments = "";

            foreach (string identifier in symbolTable.Keys)
            {
                if (constantValuedIdentifiers.Contains(identifier)) continue;

                parameters += $"{symbolTable[identifier]} {identifier}, ";
                assignments += $"{MsgAutoGenUtilities.TWO_TABS}{MsgAutoGenUtilities.ONE_TAB}this.{identifier} = {identifier};\n";
            }

            if (!string.IsNullOrEmpty(parameters))
                parameters = parameters[..^2]; // Remove trailing comma and space

            constructor += $"{MsgAutoGenUtilities.TWO_TABS}public {className}({parameters})\n"
                        + $"{MsgAutoGenUtilities.TWO_TABS}{{\n"
                        + assignments
                        + $"{MsgAutoGenUtilities.TWO_TABS}}}\n";

            return constructor;
        }
    }
    #endregion
    public class MessageParserException : Exception
    {
        public MessageParserException(string msg) : base(msg) { }
    }
}
