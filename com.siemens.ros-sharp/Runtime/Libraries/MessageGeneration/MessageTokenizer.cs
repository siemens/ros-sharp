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

Note: This file is a refactored version of the original MessageTokenizer.cs
    written by Sifan Ye, © Siemens AG, 2019
*/

using System;
using System.IO;
using System.Collections.Generic;
using System.Text.RegularExpressions;

namespace RosSharp.RosBridgeClient.MessageGeneration
{
    public class MessageTokenizer
    {
        private string inFilePath = "";
        private uint lineNum = 1;
        private readonly HashSet<string> builtInTypes;
        public MessageTokenizer(string inFilePath, HashSet<string> builtInTypes)
        {
            this.inFilePath = inFilePath;
            this.builtInTypes = builtInTypes;
        }

        /// <summary>
        /// Tokenizes the stream input
        /// </summary>
        /// <returns> A list of MessageTokens read from the stream </returns>
        public List<List<MessageToken>> Tokenize()
        {
            var listsOfTokens = new List<List<MessageToken>>();
            var currentTokens = new List<MessageToken>();
            listsOfTokens.Add(currentTokens);
            currentTokens.Add(new MessageToken(MessageTokenType.FilePath, inFilePath, 0));

            using (var reader = new StreamReader(inFilePath))
            {
                string line;
                while ((line = reader.ReadLine()) != null)
                {
                    string trimmed = line.Trim();
                    if (string.IsNullOrEmpty(trimmed))
                    {
                        lineNum++;
                        continue;
                    }
                    if (trimmed.StartsWith("#"))
                    {
                        currentTokens.Add(new MessageToken(MessageTokenType.Comment, trimmed.Substring(1), lineNum)); // (1).Trim()
                        lineNum++;
                        continue;
                    }
                    if (trimmed == "---")
                    {
                        currentTokens = new List<MessageToken>();
                        listsOfTokens.Add(currentTokens);
                        currentTokens.Add(new MessageToken(MessageTokenType.FilePath, inFilePath, 0));
                        lineNum++;
                        continue;
                    }
                    try
                    {
                        ParseDeclarationLine(trimmed, currentTokens);
                    }
                    catch (Exception ex)
                    {
                        throw new MessageTokenizerException($"Error on line {lineNum}: {ex.Message} ({inFilePath}:{lineNum})");
                    }
                    lineNum++;
                }
            }
            return listsOfTokens;
        }

        private static readonly Regex declarationRegex = new Regex(
            // type, optional array, identifier, optional (= value or value), optional # comment
            // e.g. int32[<=10] my_array = 5 # comment
            //      ^type      ^array   ^id  ^val   ^comment
            @"^(?<type>[A-Za-z_\/][A-Za-z0-9_\/]*)(?<strbound><=\d+)?(?<array>\[[^\]]*\])?\s+" +
            @"(?<identifier>[A-Za-z_][A-Za-z0-9_]*)" +                          // IDENTIFIER
            @"(?:\s*=\s*(?<constval>[^#]+?)|\s+(?<defval>[^#=\s][^#]*?))?" +    // VALUE (CONSTAT OR INITIAL)
            @"(?:\s*#\s*(?<comment>.*))?$",                                     // COMMENT 
            RegexOptions.Compiled
        );

        private void ParseDeclarationLine(string line, List<MessageToken> tokens)
        {
            var match = declarationRegex.Match(line);
            if (!match.Success)
                throw new MessageTokenizerException("Invalid declaration syntax");

            string typePart = match.Groups["type"].Value;
            string arrayPart = match.Groups["array"].Success ? match.Groups["array"].Value : null;
            string strBound = match.Groups["strbound"].Success ? match.Groups["strbound"].Value : null;
            string identifier = match.Groups["identifier"].Value;
            string constval = match.Groups["constval"].Success ? match.Groups["constval"].Value.Trim() : null;
            string defval = match.Groups["defval"].Success ? match.Groups["defval"].Value.Trim() : null;
            string comment = match.Groups["comment"].Success ? match.Groups["comment"].Value.Trim() : null;

            AddTypeToken(typePart, strBound, tokens);
            if (arrayPart != null)
                tokens.Add(ParseArrayToken(arrayPart));

            tokens.Add(new MessageToken(MessageTokenType.Identifier, identifier, lineNum));
            if (constval != null)
                tokens.Add(new MessageToken(MessageTokenType.ConstantValueDeclaration, constval, lineNum));
            else if (defval != null)
                tokens.Add(new MessageToken(MessageTokenType.DefaultValueDeclaration, defval, lineNum));
            if (!string.IsNullOrEmpty(comment))
                tokens.Add(new MessageToken(MessageTokenType.Comment, comment, lineNum));
        }

        private void AddTypeToken(string typeStr, string strBound, List<MessageToken> tokens)
        {
            // if (!string.IsNullOrEmpty(strBound))
            // Debug.LogWarning($"StrBound '{strBound}' is not used in type tokenization. Type: {typeStr} (line {lineNum})");

            if (builtInTypes.Contains(typeStr))
                tokens.Add(new MessageToken(MessageTokenType.BuiltInType, typeStr, lineNum));

            else if (typeStr == "Header")
                tokens.Add(new MessageToken(MessageTokenType.Header, typeStr, lineNum));

            else
                tokens.Add(new MessageToken(MessageTokenType.DefinedType, typeStr, lineNum));
        }

        private static readonly Regex arrayRegex = new Regex(
            @"^\[(?<bound><=|>=)?(?<size>\d*)\]$",
            RegexOptions.Compiled
        );

        private MessageToken ParseArrayToken(string arrayStr)
        {
            var match = arrayRegex.Match(arrayStr);
            if (!match.Success)
                throw new MessageTokenizerException($"Invalid array declaration: {arrayStr}");

            string bound = match.Groups["bound"].Success ? match.Groups["bound"].Value : null;
            string size = match.Groups["size"].Value;

            if (string.IsNullOrEmpty(bound) && string.IsNullOrEmpty(size))
                return new MessageToken(MessageTokenType.VariableSizeArray, "", lineNum);
            if (!string.IsNullOrEmpty(bound) && !string.IsNullOrEmpty(size))
                return new MessageToken(MessageTokenType.BoundedVariableSizeArray, bound + "~" + size, lineNum);
            if (!string.IsNullOrEmpty(size))
                return new MessageToken(MessageTokenType.FixedSizeArray, size, lineNum);
            throw new MessageTokenizerException($"Invalid array declaration: {arrayStr}");
        }
    }

    public class MessageTokenizerException : Exception
    {
        public MessageTokenizerException(string msg) : base(msg) { }
    }
}