import google.generativeai as genai

# Inserisci la tua chiave
GEMINI_API_KEY = os.getenv("GOOGLE_API_KEY")
genai.configure(api_key=GEMINI_API_KEY)

print("Modelli disponibili che supportano 'generateContent':")
for m in genai.list_models():
  if 'generateContent' in m.supported_generation_methods:
    print(m.name)