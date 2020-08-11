#include <algorithm>
#include <cassert>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>

enum TokenType {
  DECL_INT,
  DECL_FLOAT,
  CMD_PRINT,
  CONST_INT,
  CONST_FLOAT,
  PAREN_L,
  PAREN_R,
  IDENTIFIER,
  BIN_OP_ADD,
  BIN_OP_SUB,
  BIN_OP_MUL,
  BIN_OP_DIV,
  BIN_OP_ASSIGN,
  END_OF_FILE
};

struct Token {
  TokenType Type;
  std::string Value;
};

template <typename... ArgsT> void EmitErrorImpl(ArgsT &&... Args);
template <> void EmitErrorImpl<>() {}

template <typename HeadT, typename... TailT>
void EmitErrorImpl(HeadT &&H, TailT &&... T) {
  std::cerr << H;
  EmitErrorImpl(std::forward<TailT>(T)...);
}

template <typename... ArgsT>
[[noreturn]] void EmitError(std::string E, ArgsT &&... Args) {
  std::cerr << "[Error] " << E << ": ";
  EmitErrorImpl(std::forward<ArgsT>(Args)...);
  std::cerr << "\n";
  exit(1);
}

class Tokenizer {
  std::ifstream IFS;
  Token CurToken;

  Token getNextToken();
  Token getNumericToken(char C);
  Token getDeclOrIDToken(char C);
  Token getParenOrOpToken(char C);

public:
  Tokenizer(std::ifstream &&S) : IFS(std::move(S)), CurToken(getNextToken()) {}

  operator bool() const { return CurToken.Type != END_OF_FILE; }
  Token readToken();
  const Token &peekToken();
};

Token Tokenizer::getNumericToken(char C) {
  Token Tok;
  while (isdigit(C)) {
    Tok.Value += C;
    C = IFS.get();
  }
  if (C != '.') {
    Tok.Type = CONST_INT;
  } else {
    Tok.Type = CONST_FLOAT;
    Tok.Value += '.';
    while (C = IFS.get(), isdigit(C))
      Tok.Value += C;
  }
  IFS.unget();
  return Tok;
}

Token Tokenizer::getDeclOrIDToken(char C) {
  std::string V;
  while (isalpha(C)) {
    V += C;
    C = IFS.get();
  }
  Token Tok;
  if (V.size() == 1 && V[0] == 'i' || V[0] == 'f' || V[0] == 'p') {
    Tok.Type = V[0] == 'i' ? DECL_INT : V[0] == 'f' ? DECL_FLOAT : CMD_PRINT;
  } else {
    Tok.Type = IDENTIFIER;
    Tok.Value = std::move(V);
  }
  IFS.unget();
  return Tok;
}

Token Tokenizer::getParenOrOpToken(char C) {
  switch (C) {
  case '+':
    return Token{BIN_OP_ADD};
  case '-':
    return Token{BIN_OP_SUB};
  case '*':
    return Token{BIN_OP_MUL};
  case '/':
    return Token{BIN_OP_DIV};
  case '=':
    return Token{BIN_OP_ASSIGN};
  case '(':
    return Token{PAREN_L};
  case ')':
    return Token{PAREN_R};
  default:
    EmitError("Tokenizer", "expecting '+', '-', '*', '/' or '=', but '", C,
              "' found.");
  }
}

Token Tokenizer::getNextToken() {
  char C = IFS.get();
  while (isspace(C))
    C = IFS.get();

  if (C == EOF)
    return Token{END_OF_FILE};
  if (isdigit(C))
    return getNumericToken(C);
  if (isalpha(C))
    return getDeclOrIDToken(C);

  return getParenOrOpToken(C);
}

const Token &Tokenizer::peekToken() { return CurToken; }

Token Tokenizer::readToken() {
  Token Result(std::move(CurToken));
  CurToken = getNextToken();
  return Result;
}

std::ostream &operator<<(std::ostream &S, TokenType T) {
  if (T == DECL_INT)
    return S << "DECL_INT";
  if (T == DECL_FLOAT)
    return S << "DECL_FLOAT";
  if (T == CMD_PRINT)
    return S << "CMD_PRINT";
  return S;
}

enum VariableType { VAR_INT, VAR_FLOAT };

enum ASTNodeType {
  PROGRAM_NODE,
  DECLARATION_NODE,
  ASSIGNMENT_NODE,
  PRINTSTMT_NODE,
  IDENTIFIER_NODE,
  CONST_INT_NODE,
  CONST_FLOAT_NODE,
  BIN_ADD_NODE,
  BIN_SUB_NODE,
  BIN_MUL_NODE,
  BIN_DIV_NODE
};

struct AST {
  ASTNodeType Type;
  std::vector<AST *> SubTree;
  std::variant<int, float, size_t> Value;

  AST(ASTNodeType T) : Type(T) {}
  AST(ASTNodeType T, AST *Child) : Type(T), SubTree({Child}) {}
};

class SymbolTable {
  std::unordered_map<std::string, size_t> VarID;
  std::vector<VariableType> VarType;

public:
  size_t declareSymbol(const std::string &ID, TokenType DeclType) {
    if (VarID.find(ID) != VarID.end())
      EmitError("SymbolTable", "redeclaration of variable ", ID);
    size_t Res = (VarID[ID] = VarType.size());
    VarType.push_back((DeclType == DECL_INT ? VAR_INT : VAR_FLOAT));
    return Res;
  }

  size_t getVarID(const std::string &ID) {
    auto Ptr = VarID.find(ID);
    if (Ptr == VarID.end())
      EmitError("SymbolTable", "use of undeclared variable ", ID);
    return Ptr->second;
  }
};

class Parser {
  Tokenizer TK;
  SymbolTable ST;
  void reduceStack(std::vector<AST *> &Stack);
  AST *parseStatement();
  AST *parseAssignment(const std::string &ID);
  AST *parseDeclaration(TokenType DeclType);
  AST *parseExpression();

public:
  Parser(std::ifstream &&S) : TK(std::move(S)) {}
  AST *parse();
};

AST *Parser::parse() {
  AST *Node = new AST(PROGRAM_NODE);
  while (TK)
    Node->SubTree.push_back(parseStatement());
  return Node;
}

AST *Parser::parseStatement() {
  Token Tok = TK.readToken();
  if (Tok.Type == DECL_INT || Tok.Type == DECL_FLOAT)
    return parseDeclaration(Tok.Type);
  if (Tok.Type == IDENTIFIER)
    return parseAssignment(Tok.Value);

  if (Tok.Type != CMD_PRINT)
    EmitError("Parser",
              "expecting DECL_INT, DECL_FLOAT, IDENTIFIER or CMD_PRINT, but ",
              Tok.Type, " found.");
  Tok = TK.readToken();
  if (Tok.Type != IDENTIFIER)
    EmitError("Parser", "expecting IDENTIFIER, but ", Tok.Type, " found.");

  size_t Var = ST.getVarID(Tok.Value);
  AST *Node = new AST(PRINTSTMT_NODE);
  Node->Value = Var;
  return Node;
}

AST *Parser::parseDeclaration(TokenType DeclType) {
  Token Tok = TK.readToken();
  if (Tok.Type != IDENTIFIER)
    EmitError("Parser", "expecting IDENTIFIER, but ", DeclType, " found.");
  AST *Node = new AST(DECLARATION_NODE);
  Node->Value = ST.declareSymbol(Tok.Value, DeclType);
  return Node;
}

AST *Parser::parseAssignment(const std::string &ID) {
  size_t Var = ST.getVarID(ID);
  if (auto T = TK.readToken().Type; T != BIN_OP_ASSIGN)
    EmitError("Parser", "expecting BIN_OP_ASSIGN, but ", T, " found.");

  return new AST(ASSIGNMENT_NODE, parseExpression());
}

void Parser::reduceStack(std::vector<AST *> &Stack) {
  std::vector<AST *> Nodes;
  while (Stack.size() >= 2 && (Stack[Stack.size() - 2]->Type == BIN_MUL_NODE ||
                               Stack[Stack.size() - 2]->Type == BIN_DIV_NODE)) {
    Nodes.push_back(Stack.back());
    Stack.pop_back();
    Nodes.push_back(Stack.back());
    Stack.pop_back();
  }
  assert(!Stack.empty());
  Nodes.push_back(Stack.back());
  Stack.pop_back();
  std::reverse(Nodes.begin(), Nodes.end());
  AST *Prev = Nodes[0];
  for (size_t i = 1; i < Nodes.size(); i += 2) {
    assert(Nodes[i]->Type == BIN_MUL_NODE || Nodes[i]->Type == BIN_DIV_NODE);
    Nodes[i]->SubTree = {Prev, Nodes[i + 1]};
    Prev = Nodes[i];
  }
  Stack.push_back(Prev);
}

AST *Parser::parseExpression() {
  std::vector<AST *> Stack;
  bool IsValue = true, Terminated = false;
  while (!Terminated) {
    TokenType Type = TK.peekToken().Type;
    if (IsValue) {
      switch (Type) {
      case PAREN_L: {
        TK.readToken();
        Stack.push_back(parseExpression());
        if (auto T = TK.readToken().Type; T != PAREN_R)
          EmitError("Parser", "expecting PAREN_R, but ", T, " found.");
        break;
      }
      case IDENTIFIER: {
        std::string ID = TK.readToken().Value;
        AST *Node = new AST(IDENTIFIER_NODE);
        Node->Value = ST.getVarID(ID);
        Stack.push_back(Node);
        break;
      }
      case CONST_INT: {
        std::string Value = TK.readToken().Value;
        AST *Node = new AST(CONST_INT_NODE);
        Node->Value = std::stoi(Value.c_str());
        Stack.push_back(Node);
        break;
      }
      case CONST_FLOAT: {
        std::string Value = TK.readToken().Value;
        AST *Node = new AST(CONST_FLOAT_NODE);
        Node->Value = std::stof(Value.c_str());
        Stack.push_back(Node);
        break;
      }
      default:
        EmitError("Parser", "unexpected token type.");
      }
    } else {
      switch (Type) {
      case BIN_OP_ADD:
      case BIN_OP_SUB:
        TK.readToken();
        reduceStack(Stack);
        Stack.push_back(
            new AST(Type == BIN_OP_ADD ? BIN_ADD_NODE : BIN_SUB_NODE));
        break;
      case BIN_OP_MUL:
      case BIN_OP_DIV:
        TK.readToken();
        Stack.push_back(
            new AST(Type == BIN_OP_MUL ? BIN_MUL_NODE : BIN_DIV_NODE));
        break;
      default:
        Terminated = true;
        break;
      }
    }
    IsValue = !IsValue;
  }
  reduceStack(Stack);
  assert(Stack.size() % 2 == 1);
  AST *Prev = Stack[0];
  for (size_t i = 1; i < Stack.size(); i += 2) {
    assert(Stack[i]->Type == BIN_ADD_NODE || Stack[i]->Type == BIN_SUB_NODE);
    Stack[i]->SubTree = {Prev, Stack[i + 1]};
    Prev = Stack[i];
  }
  return Prev;
}

void generateAssignment(AST *Node) {
  
}

void generateStatement(AST *Stmt) {
  switch (Stmt->Type) {
    case ASSIGNMENT_NODE:
      generateAssignment(Stmt);
  }
}

void generateProgarm(AST *Program) {
  for (AST *Node : Program->SubTree)
    generateStatement(Node);
}

int main(int argc, const char **argv) {
  if (argc != 3) {
    std::cerr << "[Usage] " << argv[0] << " source_file target_file\n";
    exit(1);
  }
  std::ifstream Source(argv[1]);
  std::ofstream Target(argv[2]);
  Parser P(std::move(Source));
  AST *Program = P.parse();
  generateProgram(Program);
}
